# carlaviz -> the ground-truth (GT) sidecar for viz (perotheia fork, branch day0)

Upstream: mjxu96/carlaviz @ f4e026d (MIT, Copyright (c) 2020-2023 Minjun Xu). The LICENSE and the upstream
history are kept. Direction: pascal for the user, 2026-11-13 15:15, on s-db2a0494.

## What it becomes

carlaviz's backend becomes `carla_gt`: a separate process with a READ-ONLY CARLA client.
- It never spawns, changes settings, ticks or destroys anything.
- It samples the client's world snapshot at up to 10 Hz and reads the map once.
- It serves ground truth over a websocket. viz's vizd connects as a client and puts the data into its Scene.

## Deployment and measurement

- It is test-only: it runs in carla_ws's `RIG_VIZ` layer next to `VizApp`, using bond001's `test_only` and
  `ports` (its own port, proposed 8091).
- It is an extra CARLA client, so the server frame rate is measured with and without it in bond003's I3 runs.

## Plan

1. **Strip what viz does not use.**
   - The frontend: viz keeps its own copy.
   - The sensor `Listen()`s: cameras come from the stack's RDS.
   - `WaitForTick` per frame and per-frame `GetActor` calls: replaced by snapshot sampling plus an actor cache.
   - Conan, Docker, and the stop-sign and 3D-traffic-light RPCs.
2. **Build from source with Bazel and the host compiler.**
   - LibCarla 0.9.15 comes from carla_ws's vendored client, as `vizd_carla` does today.
   - websocketpp comes from viz's vendored copy.
   - No Conan and no prebuilt binaries.
3. **The wire is viz's GT spec draft 1** (s-db2a0494 `specs/gt-wire-draft1.md`): JSON messages hello, map, actors,
   frame and stats, with the map frame in ENU. The serving side is carlaviz's backend loop. It builds nothing
   when no client is connected, caps the rate, and drops frames for a slow client.
4. **viz moves to it.** viz gets a websocket client source, and `vizd_carla` with its LibCarla link is deleted.
   viz's `carla_source.cc` extraction code, which already follows carlaviz's map walk, moves here.
