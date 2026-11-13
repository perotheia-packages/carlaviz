# websocketpp (vendored, header-only)

- Source: https://github.com/zaphoyd/websocketpp, branch `develop`, commit
  `bce258674b2d544f981da4bf149244a76e8126c9` (2026-05-04). BSD-3-Clause (COPYING).
- Why not the 0.8.2 tag carlaviz/Conan use: 0.8.2 does not compile as C++20 with gcc 11 ("template-id not allowed
  for constructor/destructor"); develop does. Only the `websocketpp/` headers are kept (no tests, examples, CMake).
- Used with standalone Asio from the host (libasio-dev 1.18.1, header-only), no TLS.
