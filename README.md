# carla_gt: carlaviz revamped into a read-only CARLA ground-truth sidecar

This is the perotheia fork of [mjxu96/carlaviz](https://github.com/mjxu96/carlaviz), branch `day0`, forked at
f4e026d. The upstream history and the MIT licence are kept (LICENSE, Copyright (c) 2020-2023 Minjun Xu). Only
the backend's job survives: reading CARLA's ground truth. It is served to viz's `vizd`
(perotheia-packages/viz), which draws it. The frontend lives on in viz. Why and how: REVAMP.md.

## What it does

`carla_gt` is its own process with a **read-only** CARLA client:
- It never spawns, changes settings, ticks, `Listen()`s to a sensor or destroys anything.
- It samples the client's world snapshot at up to 10 Hz and reads the map once per episode.
- It fetches the actor list once, and again only when a new actor id appears.
- It serves ground truth on `ws://<bind>:8091/` as JSON (viz's GT spec draft 1): `hello`, `map` (lane centres
  and borders, the traffic lights' trigger areas), `actors` (static descriptions), `frame` (pose, velocity and
  acceleration of every vehicle and walker; light states), and `stats` (1 Hz).
- Everything is in map ENU: CARLA's (x, -y, z), with yaw and pitch negated, in radians.

**Zero cost with nobody watching:** it joins CARLA only while a client is connected, and leaves CARLA's tick
stream `--idle_s` (10) seconds after the last one goes. It builds no frame when no client is connected, and a
slow client loses frames (counted) instead of queueing them. It runs at nice 19 + SCHED_IDLE.

It is a **test-only** component. carla_ws's `RIG_VIZ` layer adds it next to viz's VizApp, with its own port; it
never goes into a production image.

## Build (from source, host compiler; no Conan, no prebuilt)

```
tools/unpack_carla.sh                 # carla_ws's vendored LibCarla 0.9.15 client -> third_party/carla/install
bazel build //backend:carla_gt        # websocketpp from ../viz/third_party/websocketpp; nlohmann/json from the host
bazel-bin/backend/carla_gt --carla_host=127.0.0.1 --carla_port=2000 --port=8091
```

## What was removed from upstream

- The React frontend: viz keeps the one it builds.
- The sensor listeners: cameras come from the stack's own RDS streams.
- The XVIZ translation: vizd does that.
- `WaitForTick` per frame and `GetActor` per actor per frame.
- Conan, CMake, Docker, the CI workflows, and the examples (they spawn actors).
