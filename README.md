# RealSense camera stream

How frames from the head-mounted Intel RealSense D435i get from the camera to
the PC.

Scope: the streaming pipeline only — `realsense_server.py`, its clients, and
the three scripts that manage it. The AprilTag detectors, viewers and
calibration tools in this folder are consumers of the stream and are documented
in their own docstrings.

The one thing to understand first:

> **The camera is not plugged into the PC that processes its images.**

It is plugged into a *camera host* — a machine with a USB 3.0 port, carried by
or near the robot. That was originally the Go2's own CPU; it can equally be a
Raspberry Pi wired to the camera instead. The frames reach the PC over the
network, and nothing here assumes which machine is on the other end.

```
D435i ──USB 3.0──▶ camera host ──TCP:9999──▶ PC
                   realsense_server.py        one client only
```

---

## The one-client rule

`realsense_server.py` accepts **exactly one client at a time** (`listen(1)`).

This single constraint explains most of the surprising behaviour around the
stream: start a second consumer and it silently gets nothing, while the first
one keeps working. Three things can be that client, and only one may run:

| Client | Where it lives | What it is for |
|---|---|---|
| `realsense_client.py` | this folder | Desktop viewer, latency measurement |
| `apriltag_tcp_detector_node.py` | this folder | Tag detection → ROS topics + TF |
| `head_camera_tcp_bridge.py` | `../go2_perception/` | Depth → PointCloud2 for Nav2 |

This is why `start_realsense.sh` defaults to `RUN_CLIENT=false`: it brings the
server up and stops, leaving the slot free for whichever consumer you want.

---

## Configuration: one place, one variable

`camera_env.sh` and `../go2_perception/camera_config.py` are deliberate twins.
They read the **same environment variables**, so the shell scripts, the ROS
nodes and the operator GUI all follow a single setting:

```bash
export GO2_CAMERA_HOST=192.168.123.90   # machine running realsense_server.py
export GO2_CAMERA_USER=sf-system        # SSH login on that machine
export GO2_CAMERA_PORT=9999             # TCP port
```

Set these rather than editing defaults in either file. The values previously
lived as separate copies in each script, which drifted the moment the camera
moved hosts: the start script pointed at the new machine while stop and status
still talked to the old one, so "stop" quietly did nothing and "status" always
read dead.

`GO2_CAMERA_USER` travels with the host on purpose — the login on a Raspberry
Pi is not the login on a Go2, so overriding one without the other just logs in
as the wrong account.

---

## Usage

One-time setup, so nothing ever prompts for a password:

```bash
ssh-copy-id "$GO2_CAMERA_USER@$GO2_CAMERA_HOST"
```

Then:

```bash
# Deploy + start the server on the camera host, leaving the client slot free
./start_realsense.sh

# Take the slot with the AprilTag detector
ros2 launch go2_perception apriltag_checkpoint_tcp.launch.py

# — or instead, take it with the desktop viewer —
RUN_CLIENT=true ./start_realsense.sh
```

Check and stop:

```bash
./status_realsense.sh    # read-only, safe any time
./stop_realsense.sh
```

| File | Runs on | Purpose |
|---|---|---|
| `realsense_server.py` | camera host | Opens the D435i, serves frames over TCP |
| `realsense_client.py` | PC | Desktop viewer; logs per-frame latency to CSV |
| `start_realsense.sh` | PC | Deploys + starts the server over SSH, waits for ready |
| `stop_realsense.sh` | PC | Stops it cleanly (TERM, then KILL, then waits for the port) |
| `status_realsense.sh` | PC | Reports whether it is up. Changes nothing |
| `camera_env.sh` | PC | Shared config for the three scripts above |

### What `start_realsense.sh` actually does

1. **Syncs the server.** `scp`s the local `realsense_server.py` to the camera
   host, so the host always runs the version in this repo rather than an older
   copy someone left there.
2. **Restarts it over SSH.** Checks whether a healthy server is already running
   — both that the process exists *and* that the port is genuinely listening,
   since a dead process with a stale port entry would otherwise look fine. Kills
   any old one and waits for the USB device to be released before relaunching.
3. **Waits for readiness.** Polls the port from the PC side until it actually
   accepts a connection, rather than assuming the SSH command's success meant
   the camera came up.
4. **Optionally starts the viewer**, per `RUN_CLIENT`.

Steps 2 and 3 are why the script is longer than it looks like it should be: a
camera that half-starts is much worse than one that fails loudly.

---

## Wire protocol

Useful when debugging a partial frame or writing a new client. TCP carries no
message boundaries, so every block is length-prefixed and the whole packet is
prefixed again with its total size.

```
[t_capture  8B double]   server clock, when the frame was grabbed
[t_encoded  8B double]   server clock, when encoding finished
[len 4B][colour JPEG        ]
[len 4B][depth viz JPEG     ]   empty in colour-only mode
[len 4B][depth raw PNG uint16]  empty in colour-only mode
[height 2B][width 2B]
[intrinsics 36B]         fx, fy, ppx, ppy, k1, k2, p1, p2, k3
```

Three design points worth knowing:

- **Two depth images.** The JPEG is lossy and for human eyes. The PNG is
  lossless uint16 millimetres and is what code reads for real distances —
  JPEG artefacts would corrupt the numbers.
- **Intrinsics last.** Appended at the end so older clients that stop reading
  after the metadata ignore them harmlessly. New consumers read `packet[-36:]`.
- **Colour-only mode.** A client may send a single byte `C` after the
  handshake; the server then skips depth encoding entirely. The uint16 PNG is
  the slowest encode on an ARM CPU and roughly 80% of the packet bytes, so this
  matters a great deal on a weaker camera host. Clients that send nothing get
  the full packet, so old clients keep working unchanged.

### Clock sync

Server and client clocks are not synchronised, so measuring "capture →
display" latency means comparing timestamps from two different clocks.

The client pings ten times, assumes each reply landed at the midpoint of its
round trip, and takes the **median** offset to reject outliers. The midpoint
assumption holds only if the path is symmetric, which WiFi does not guarantee —
so the client clamps negative results to zero rather than reporting a
physically impossible latency.

Latency is then reported split three ways, which is the point of the exercise:

| Metric | Meaning | What a high value means |
|---|---|---|
| `encode_ms` | `t_encoded - t_capture` | Camera host CPU is the bottleneck |
| `net_ms` | encode done → client receive | The network is the bottleneck |
| `total` | capture → client receive | Sum of both |

`encode_ms` needs no clock sync at all — both timestamps come from the server.

---

## Troubleshooting

**Black stream, no error.** Almost always the one-client rule. Something else
already holds the slot — often a consumer that outlived its parent process.
Check with `./status_realsense.sh`, then `./stop_realsense.sh` and start again.

**`pipeline.start` fails with the device busy.** The USB device has not been
released yet by the previous process. The server already retries four times
with a two-second gap, and `start_realsense.sh` waits after killing an old
server; if it still fails, unplug and replug the camera.

**Everything is slow, and the network looks fine.** On a Raspberry Pi camera
host, check power before suspecting the link:

```bash
vcgencmd get_throttled    # 0x0 means healthy
```

A Pi 4 plus a D435i can exceed a 3 A supply. Undervoltage throttles the CPU,
which slows the depth encode and drops the frame rate — a symptom that looks
exactly like a network problem but is not one. Consider colour-only mode, a
powered USB hub, or a separate supply.

**`pyrealsense2` will not install on the camera host.** There is no official
ARM64 wheel. `librealsense` has to be built from source with
`-DBUILD_PYTHON_BINDINGS=ON`, which takes well under an hour on a Pi 4 but is
not a `pip install`.

---

## Related

- `../go2_perception/camera_config.py` — the Python half of the shared config
- `../../go2_backend/api/camera_runner.py` — how the operator GUI drives this
- `../launch/apriltag_checkpoint_tcp.launch.py` — the usual consumer
