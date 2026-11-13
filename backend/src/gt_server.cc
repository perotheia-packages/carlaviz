// carla_gt: the ground-truth websocket server. See gt_server.h.
#include "carlaviz/gt/gt_server.h"

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <cstdio>
#include <mutex>
#include <set>
#include <thread>

namespace carlaviz::gt {

namespace {
using Server = websocketpp::server<websocketpp::config::asio>;
using Hdl = websocketpp::connection_hdl;
}  // namespace

struct GtServer::Impl {
  Options opt;
  Server srv;
  std::thread io;
  mutable std::mutex mu;   // clients + the cached init messages
  std::set<Hdl, std::owner_less<Hdl>> clients;
  std::string hello, map, actors;
  std::atomic<uint64_t> sent{0}, skipped{0}, bytes{0};

  // I/O thread only.
  void SendTo(Hdl h, const std::string& m, bool droppable) {
    websocketpp::lib::error_code ec;
    auto con = srv.get_con_from_hdl(h, ec);
    if (ec || !con) return;
    if (droppable && con->get_buffered_amount() > opt.max_buffered) {
      skipped.fetch_add(1);
      return;
    }
    ec = con->send(m.data(), m.size(), websocketpp::frame::opcode::text);
    if (!ec) {
      sent.fetch_add(1);
      bytes.fetch_add(m.size());
    }
  }

  void Broadcast(std::shared_ptr<const std::string> m, bool droppable) {
    asio::post(srv.get_io_service(), [this, m, droppable] {
      std::set<Hdl, std::owner_less<Hdl>> cs;
      {
        std::lock_guard<std::mutex> lk(mu);
        cs = clients;
      }
      for (const auto& h : cs) SendTo(h, *m, droppable);
    });
  }

  void SetCached(std::string Impl::*slot, std::string m) {
    {
      std::lock_guard<std::mutex> lk(mu);
      this->*slot = m;
    }
    if (!m.empty()) Broadcast(std::make_shared<const std::string>(std::move(m)), false);
  }
};

GtServer::GtServer(Options o) : impl_(std::make_unique<Impl>()) { impl_->opt = std::move(o); }

GtServer::~GtServer() { Stop(); }

bool GtServer::Start() {
  Impl* I = impl_.get();
  try {
    I->srv.init_asio();
    I->srv.clear_access_channels(websocketpp::log::alevel::all);
    I->srv.clear_error_channels(websocketpp::log::elevel::all);
    I->srv.set_reuse_addr(true);
    I->srv.set_open_handler([I](Hdl h) {
      std::string hello, map, actors;
      size_t n;
      {
        std::lock_guard<std::mutex> lk(I->mu);
        I->clients.insert(h);
        hello = I->hello;
        map = I->map;
        actors = I->actors;
        n = I->clients.size();
      }
      for (const auto* m : {&hello, &map, &actors})
        if (!m->empty()) I->SendTo(h, *m, false);
      std::fprintf(stderr, "[carla_gt] client connected (%zu)\n", n);
    });
    auto gone = [I](Hdl h) {
      size_t n;
      {
        std::lock_guard<std::mutex> lk(I->mu);
        I->clients.erase(h);
        n = I->clients.size();
      }
      std::fprintf(stderr, "[carla_gt] client gone (%zu)\n", n);
    };
    I->srv.set_close_handler(gone);
    I->srv.set_fail_handler(gone);
    I->srv.set_message_handler([](Hdl, Server::message_ptr) {});   // push only: input is ignored
    I->srv.listen(asio::ip::tcp::endpoint(asio::ip::make_address(I->opt.bind), I->opt.port));
    I->srv.start_accept();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[carla_gt] cannot listen on %s:%u: %s\n", I->opt.bind.c_str(), I->opt.port, e.what());
    return false;
  }
  I->io = std::thread([I] {
    try {
      I->srv.run();
    } catch (const std::exception& e) {
      std::fprintf(stderr, "[carla_gt] server stopped: %s\n", e.what());
    }
  });
  return true;
}

void GtServer::Stop() {
  if (!impl_ || !impl_->io.joinable()) return;
  impl_->srv.stop();
  impl_->io.join();
}

void GtServer::SetHello(std::string m) { impl_->SetCached(&Impl::hello, std::move(m)); }
void GtServer::SetMap(std::string m) { impl_->SetCached(&Impl::map, std::move(m)); }
void GtServer::SetActors(std::string m) { impl_->SetCached(&Impl::actors, std::move(m)); }
void GtServer::SendFrame(std::string m) { impl_->Broadcast(std::make_shared<const std::string>(std::move(m)), true); }
void GtServer::SendStats(std::string m) { impl_->Broadcast(std::make_shared<const std::string>(std::move(m)), true); }

uint64_t GtServer::Clients() const {
  std::lock_guard<std::mutex> lk(impl_->mu);
  return impl_->clients.size();
}

GtServer::Stats GtServer::GetStats() const {
  Stats s;
  s.clients = Clients();
  s.sent = impl_->sent.load();
  s.skipped = impl_->skipped.load();
  s.bytes = impl_->bytes.load();
  return s;
}

}  // namespace carlaviz::gt
