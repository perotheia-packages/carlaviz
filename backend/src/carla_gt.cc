// carla_gt: carlaviz's backend revamped into viz's READ-ONLY CARLA ground-truth sidecar (REVAMP.md).
//
// The map extraction (a topology walk at 0.5 m, lane borders at +-width/2), the box footprints and the traffic
// lights' trigger areas follow carlaviz's backend (simulators/carla.h, utils/map.h; mjxu96/carlaviz @ f4e026d,
// MIT, Copyright (c) 2020-2023 Minjun Xu). Unlike upstream it never spawns, changes settings, ticks, Listen()s
// to a sensor or destroys anything, never WaitForTick()s per frame and never fetches an actor per frame: it
// samples the client's latest world snapshot at its own rate (<= 10 Hz).
//
// ZERO COST WITH NOBODY WATCHING: it holds a CARLA client ONLY while a GT client (vizd) is connected; after
// --idle_s seconds without one it drops the CARLA client (and so leaves CARLA's tick stream).
//
// Wire: viz's GT spec draft 1 (s-db2a0494 specs/gt-wire-draft1.md): WebSocket ws://<bind>:<port>/, JSON text
// frames, server push only; messages hello, map, actors, frame, stats; map frame ENU = CARLA (x, -y, z), yaw and
// pitch negated (rad).
//
//   carla_gt [--carla_host=127.0.0.1] [--carla_port=2000] [--bind=127.0.0.1] [--port=8091] [--rate=10]
//            [--ego_role=hero] [--idle_s=10] [--priority=idle|normal]
#include "carlaviz/gt/gt_server.h"

#include <carla/client/Actor.h>
#include <carla/client/ActorList.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TrafficLight.h>
#include <carla/client/Waypoint.h>
#include <carla/client/World.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/geom/Math.h>

#include <nlohmann/json.hpp>

#include <sched.h>
#include <signal.h>
#include <sys/resource.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace cc = carla::client;
namespace cg = carla::geom;
using json = nlohmann::json;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

double r2(double v) { return std::round(v * 100.0) / 100.0; }      // 1 cm
double r4(double v) { return std::round(v * 10000.0) / 10000.0; }  // angles, rad
json enu(const cg::Location& l) { return json::array({r2(l.x), r2(-l.y), r2(l.z)}); }

std::string basename_of(const std::string& s) {
  auto p = s.rfind('/');
  return p == std::string::npos ? s : s.substr(p + 1);
}

// carlaviz's LateralShift: a point `shift` metres to the right of the waypoint (CARLA frame).
cg::Location lateral(cg::Transform t, double shift) {
  t.rotation.yaw += 90.0f;
  const auto f = t.GetForwardVector();
  return {t.location.x + float(shift) * f.x, t.location.y + float(shift) * f.y, t.location.z + float(shift) * f.z};
}

json road_lane(const carla::SharedPtr<cc::Waypoint>& start) {
  std::vector<carla::SharedPtr<cc::Waypoint>> wps{start};
  const auto road = start->GetRoadId();
  auto next = start->GetNext(0.5);
  while (!next.empty() && next[0]->GetRoadId() == road && wps.size() < 20000) {
    wps.push_back(next[0]);
    next = next[0]->GetNext(0.5);
  }
  if (wps.size() < 2) return nullptr;
  json c = json::array(), l = json::array(), r = json::array();
  for (const auto& w : wps) {
    const auto t = w->GetTransform();
    const double hw = w->GetLaneWidth() * 0.5;
    l.push_back(enu(lateral(t, -hw)));
    r.push_back(enu(lateral(t, hw)));
    c.push_back(enu(t.location));
  }
  return json{{"road", start->GetRoadId()}, {"lane", start->GetLaneId()}, {"junction", start->IsJunction()},
              {"center", std::move(c)}, {"left", std::move(l)}, {"right", std::move(r)}};
}

const char* light_state(carla::rpc::TrafficLightState s) {
  switch (s) {
    case carla::rpc::TrafficLightState::Red: return "red";
    case carla::rpc::TrafficLightState::Yellow: return "yellow";
    case carla::rpc::TrafficLightState::Green: return "green";
    case carla::rpc::TrafficLightState::Off: return "off";
    default: return "unknown";
  }
}

enum class Kind { OTHER, VEHICLE, WALKER, LIGHT };

struct ActorInfo {
  Kind kind = Kind::OTHER;
  bool ego = false;
  std::string type, role;
  cg::BoundingBox bb;
  carla::SharedPtr<cc::TrafficLight> light;   // LIGHT: its state is read from the local client
};

struct Flags {
  std::map<std::string, std::string> kv;
  std::string get(const std::string& k, const std::string& d) const {
    auto it = kv.find(k);
    return it == kv.end() ? d : it->second;
  }
  double num(const std::string& k, double d) const {
    auto it = kv.find(k);
    return it == kv.end() ? d : std::atof(it->second.c_str());
  }
};

// One CARLA session: from the first client to --idle_s seconds after the last one.
void session(carlaviz::gt::GtServer& srv, const Flags& f) {
  using clk = std::chrono::steady_clock;
  const std::string host = f.get("carla_host", "127.0.0.1");
  const auto port = uint16_t(f.num("carla_port", 2000));
  const std::string ego_role = f.get("ego_role", "hero");
  const double rate = std::min(10.0, std::max(0.5, f.num("rate", 10)));
  const double idle_s = f.num("idle_s", 10);

  std::fprintf(stderr, "[carla_gt] a client is watching: connecting to CARLA %s:%u (read-only)\n", host.c_str(), port);
  cc::Client client(host, port, 1);
  client.SetTimeout(std::chrono::seconds(5));
  auto world = client.GetWorld();
  std::fprintf(stderr, "[carla_gt] connected: client %s, server %s\n", client.GetClientVersion().c_str(),
               client.GetServerVersion().c_str());

  uint64_t episode = ~0ull, last_frame = 0, frames_sent = 0, actor_rpcs = 0;
  std::string map_name;
  std::unordered_map<carla::ActorId, ActorInfo> cache;
  bool actors_dirty = true;
  double fps = 0, fps_t0 = 0;
  uint64_t fps_f0 = 0;
  auto last_client = clk::now(), next = clk::now(), next_stats = clk::now();
  const auto period = std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(1.0 / rate));

  while (!g_stop) {
    next += period;
    std::this_thread::sleep_until(next);
    if (clk::now() - next > 5 * period) next = clk::now();
    if (srv.Clients() > 0) {
      last_client = clk::now();
    } else if (std::chrono::duration<double>(clk::now() - last_client).count() > idle_s) {
      std::fprintf(stderr, "[carla_gt] nobody watching for %.0f s: leaving CARLA\n", idle_s);
      srv.SetHello("");
      srv.SetMap("");
      srv.SetActors("");
      return;   // the client goes out of scope: no CARLA client while idle
    } else {
      continue;   // nobody connected right now: build nothing
    }

    const cc::WorldSnapshot snap = world.GetSnapshot();   // local: the client's latest tick
    if (snap.GetId() != episode) {
      const auto t0 = clk::now();
      auto cmap = world.GetMap();
      map_name = basename_of(cmap->GetName());
      json lanes = json::array();
      for (const auto& [a, b] : cmap->GetTopology()) {
        auto lane = road_lane(a);
        if (!lane.is_null()) lanes.push_back(std::move(lane));
      }
      episode = snap.GetId();
      cache.clear();
      actors_dirty = true;
      // Lights' trigger areas are static per episode: collect them with the first actor list below.
      json hello{{"v", 1}, {"type", "hello"}, {"server", client.GetServerVersion()},
                 {"client", client.GetClientVersion()}, {"episode", episode}, {"map", map_name},
                 {"ego_role", ego_role}, {"rate_hz", rate}};
      srv.SetHello(hello.dump());
      json m{{"v", 1}, {"type", "map"}, {"episode", episode}, {"name", map_name}, {"lanes", std::move(lanes)},
             {"lights", json::array()}};
      // (map is sent after the actor list, when the light trigger areas are known)
      std::fprintf(stderr, "[carla_gt] map %s: %zu lanes in %.0f ms\n", map_name.c_str(), m["lanes"].size(),
                   std::chrono::duration<double, std::milli>(clk::now() - t0).count());
      // New actors -> one actor-list RPC (descriptions are static per actor).
      ++actor_rpcs;
      auto actor_list = world.GetActors();   // keep the list alive for the loop
      for (const auto& a : *actor_list) {
        ActorInfo ai;
        ai.type = a->GetTypeId();
        if (ai.type.rfind("vehicle.", 0) == 0) ai.kind = Kind::VEHICLE;
        else if (ai.type.rfind("walker.pedestrian", 0) == 0) ai.kind = Kind::WALKER;
        else if (ai.type == "traffic.traffic_light") ai.kind = Kind::LIGHT;
        ai.bb = a->GetBoundingBox();
        for (const auto& at : a->GetAttributes())
          if (at.GetId() == "role_name") ai.role = at.GetValue();
        ai.ego = ai.role == ego_role;
        if (ai.kind == Kind::LIGHT) {
          ai.light = boost::dynamic_pointer_cast<cc::TrafficLight>(a);
          if (ai.light) {
            const auto v = ai.light->GetTriggerVolume().GetWorldVertices(ai.light->GetTransform());
            m["lights"].push_back({{"id", a->GetId()},
                                   {"trigger", json::array({enu(v[0]), enu(v[2]), enu(v[6]), enu(v[4])})}});
          }
        }
        cache[a->GetId()] = std::move(ai);
      }
      srv.SetMap(m.dump());
    }
    if (snap.GetFrame() == last_frame) continue;   // the server has not moved
    last_frame = snap.GetFrame();
    const double sim_s = snap.GetTimestamp().elapsed_seconds;
    const double wall_s = snap.GetTimestamp().platform_timestamp;
    if (fps_t0 == 0 || wall_s - fps_t0 > 2.0) {
      if (fps_t0 != 0) fps = double(snap.GetFrame() - fps_f0) / (wall_s - fps_t0);
      fps_t0 = wall_s;
      fps_f0 = snap.GetFrame();
    }

    // New actor ids -> one actor-list RPC; vanished ids -> a new actors message.
    bool unknown = false;
    size_t seen = 0;
    for (const auto& as : snap) {
      if (!cache.count(as.id)) { unknown = true; break; }
      ++seen;
    }
    if (unknown) {
      ++actor_rpcs;
      auto actor_list = world.GetActors();   // keep the list alive for the loop
      for (const auto& a : *actor_list) {
        if (cache.count(a->GetId())) continue;
        ActorInfo ai;
        ai.type = a->GetTypeId();
        if (ai.type.rfind("vehicle.", 0) == 0) ai.kind = Kind::VEHICLE;
        else if (ai.type.rfind("walker.pedestrian", 0) == 0) ai.kind = Kind::WALKER;
        ai.bb = a->GetBoundingBox();
        for (const auto& at : a->GetAttributes())
          if (at.GetId() == "role_name") ai.role = at.GetValue();
        ai.ego = ai.role == ego_role;
        cache[a->GetId()] = std::move(ai);
      }
      for (const auto& as : snap)   // gone before we asked: remember, so we do not ask again
        if (!cache.count(as.id)) cache[as.id] = ActorInfo{};
      actors_dirty = true;
    } else if (seen != cache.size()) {
      // ids vanished: drop them
      std::unordered_map<carla::ActorId, bool> live;
      for (const auto& as : snap) live[as.id] = true;
      for (auto it = cache.begin(); it != cache.end();)
        it = live.count(it->first) ? std::next(it) : cache.erase(it);
      actors_dirty = true;
    }
    if (actors_dirty) {
      json list = json::array();
      for (const auto& [id, ai] : cache) {
        if (ai.kind != Kind::VEHICLE && ai.kind != Kind::WALKER) continue;
        list.push_back({{"id", id},
                        {"kind", ai.kind == Kind::VEHICLE ? "vehicle" : "walker"},
                        {"type", ai.type},
                        {"role", ai.role},
                        {"ego", ai.ego},
                        {"bbox", {{"c", json::array({r2(ai.bb.location.x), r2(-ai.bb.location.y), r2(ai.bb.location.z)})},
                                  {"e", json::array({r2(ai.bb.extent.x), r2(ai.bb.extent.y), r2(ai.bb.extent.z)})}}}});
      }
      srv.SetActors(json{{"v", 1}, {"type", "actors"}, {"list", std::move(list)}}.dump());
      actors_dirty = false;
    }

    json actors = json::array(), lights = json::array();
    for (const auto& as : snap) {
      auto it = cache.find(as.id);
      if (it == cache.end()) continue;
      const ActorInfo& ai = it->second;
      if (ai.kind == Kind::LIGHT) {
        if (ai.light) lights.push_back(json::array({as.id, light_state(ai.light->GetState())}));
        continue;
      }
      if (ai.kind != Kind::VEHICLE && ai.kind != Kind::WALKER) continue;
      const auto& t = as.transform;
      actors.push_back(json::array({as.id, r2(t.location.x), r2(-t.location.y), r2(t.location.z),
                                    r4(cg::Math::ToRadians(t.rotation.roll)), r4(-cg::Math::ToRadians(t.rotation.pitch)),
                                    r4(-cg::Math::ToRadians(t.rotation.yaw)), r2(as.velocity.x), r2(-as.velocity.y),
                                    r2(as.velocity.z), r2(as.acceleration.x), r2(-as.acceleration.y),
                                    r2(as.acceleration.z)}));
    }
    srv.SendFrame(json{{"v", 1}, {"type", "frame"}, {"frame", snap.GetFrame()}, {"sim_s", sim_s},
                       {"wall_s", wall_s}, {"actors", std::move(actors)}, {"lights", std::move(lights)}}
                      .dump());
    ++frames_sent;
    if (clk::now() >= next_stats) {
      next_stats = clk::now() + std::chrono::seconds(1);
      const auto st = srv.GetStats();
      srv.SendStats(json{{"v", 1}, {"type", "stats"}, {"server_fps", r2(fps)}, {"sent", frames_sent},
                         {"skipped_slow_client", st.skipped}, {"actor_list_rpcs", actor_rpcs},
                         {"clients", st.clients}}
                        .dump());
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  Flags f;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      std::puts("carla_gt [--carla_host=127.0.0.1] [--carla_port=2000] [--bind=127.0.0.1] [--port=8091]\n"
                "         [--rate=10] [--ego_role=hero] [--idle_s=10] [--priority=idle|normal]");
      return 0;
    }
    if (a.rfind("--", 0) != 0) continue;
    a = a.substr(2);
    const auto eq = a.find('=');
    f.kv[a.substr(0, eq)] = eq == std::string::npos ? "" : a.substr(eq + 1);
  }
  if (f.get("priority", "idle") == "idle") {   // every thread created after this inherits it
    if (setpriority(PRIO_PROCESS, 0, 19) != 0) std::perror("[carla_gt] setpriority");
    sched_param sp{};
    if (sched_setscheduler(0, SCHED_IDLE, &sp) != 0) std::perror("[carla_gt] SCHED_IDLE");
  }
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);

  carlaviz::gt::GtServer::Options so;
  so.bind = f.get("bind", "127.0.0.1");
  so.port = uint16_t(f.num("port", 8091));
  carlaviz::gt::GtServer srv(so);
  if (!srv.Start()) return 1;
  std::fprintf(stderr, "[carla_gt] ws://%s:%u/ (read-only CARLA GT; CARLA is joined only while a client watches)\n",
               so.bind.c_str(), so.port);

  while (!g_stop) {
    if (srv.Clients() == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }
    try {
      session(srv, f);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "[carla_gt] %s; retrying in 2 s\n", e.what());
      srv.SetHello("");
      srv.SetMap("");
      srv.SetActors("");
      for (int i = 0; i < 20 && !g_stop; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  std::fprintf(stderr, "[carla_gt] stopping\n");
  srv.Stop();
  return 0;
}
