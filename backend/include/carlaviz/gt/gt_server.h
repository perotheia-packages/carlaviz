// carla_gt: the ground-truth websocket server (push only), after carlaviz's connectors/websocket.h.
// Copyright (c) 2020-2023 Minjun Xu (carlaviz, MIT); revamp for viz (perotheia), 2026.
//
// Clients connect to ws://<bind>:<port>/gt. A new client first receives the cached hello, map and actors
// messages (in that order), then every frame. Frames are droppable: a client whose socket still holds more
// than max_buffered bytes skips the frame (counted). All socket work runs on the server's own I/O thread: a
// caller only posts.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace carlaviz::gt {

class GtServer {
 public:
  struct Options {
    std::string bind = "127.0.0.1";
    uint16_t port = 8091;
    size_t max_buffered = 1u << 20;   // bytes queued per client before frames are skipped
  };
  struct Stats {
    uint64_t clients = 0, sent = 0, skipped = 0, bytes = 0;
  };

  explicit GtServer(Options o);
  ~GtServer();
  bool Start();   // binds and runs the I/O thread; false if the port is taken
  void Stop();

  // Cached and sent to every client now (and first to each new one). Empty string = clear.
  void SetHello(std::string m);
  void SetMap(std::string m);
  void SetActors(std::string m);
  // Sent to every client now, droppable. Not cached.
  void SendFrame(std::string m);
  void SendStats(std::string m);

  uint64_t Clients() const;
  Stats GetStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace carlaviz::gt
