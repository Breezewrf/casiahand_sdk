# Standalone dual-hand ZMQ teleop

This example runs independently of RoboJuDo. A Python publisher sends one
10-joint JSON stream per hand; the C++ subscriber connects to both streams,
drives the dual CASIA Hand-M over one serial port, and prints measured motor
positions and power feedback.

## Safety

The program controls physical hardware. Before enabling it:

- verify the serial port, left/right IDs, baud rate, and joint order
- keep the hand unloaded and clear of people and objects
- begin with the `zero` demo, then test individual motors at small positions
- be ready to stop the program and remove actuator power

The SDK clips positions to its non-negative motor ranges, but this is not a
substitute for an external emergency stop.

## Dependencies

The CASIA SDK and serial sources come from this repository. Only ZeroMQ is an
additional C++ dependency:

```bash
sudo apt-get install build-essential cmake libzmq3-dev python3-dev
python -m pip install pyzmq
```

## Port convention

This example follows the existing `dex_teleop` CASIA convention:

| Stream | Left | Right |
|---|---:|---:|
| Physical CASIA hand (`sim2real`) | `5555` | `5556` |
| Simulation hand (`sim2sim`) | `5560` | `5561` |

Because this C++ executable drives physical hands, its defaults are always
`5555/5556`. The `5560/5561` pair is documented only to prevent accidentally
connecting this hardware subscriber to the simulation streams.

## Build

From the repository root:

```bash
cmake -S examples/zmq_teleop -B build/zmq_teleop -DCMAKE_BUILD_TYPE=Release
cmake --build build/zmq_teleop -j
```

The executable is `build/zmq_teleop/casia_zmq_teleop`. This standalone build
does not require pybind11 and does not use an installed CASIA SDK.

## Run locally

The publisher owns (binds) ports 5555 and 5556. Start it first:

```bash
python examples/zmq_teleop/zmq_publisher.py \
  --endpoint-left tcp://*:5555 \
  --endpoint-right tcp://*:5556 \
  --demo zero \
  --duration 5
```

In another terminal, start the hardware subscriber:

```bash
./build/zmq_teleop/casia_zmq_teleop \
  tcp://127.0.0.1:5555 \
  tcp://127.0.0.1:5556 \
  /dev/ttyUSB0 \
  2 \
  0x20 \
  115200
```

When using WCH's out-of-tree CH341 driver, pass its real device name directly:

```bash
./build/zmq_teleop/casia_zmq_teleop \
  tcp://127.0.0.1:5555 \
  tcp://127.0.0.1:5556 \
  /dev/ttyCH341USB0 \
  2 \
  0x20 \
  115200
```

The SDK enumerates both `/dev/ttyUSB<N>` and `/dev/ttyCH341USB<N>`; no symlink
is required. Install the repository's `udev/99-casiahand-usb.rules` on the host
to make permissions persistent across USB reconnects.

Arguments are positional:

```text
casia_zmq_teleop [left_endpoint] [right_endpoint]
                  [serial_port] [left_id] [right_id] [baudrate]
```

All arguments are optional. IDs accept decimal or `0x` notation. Defaults are:

| Setting | Default |
|---|---|
| Left endpoint | `tcp://localhost:5555` |
| Right endpoint | `tcp://localhost:5556` |
| Serial port | `/dev/ttyUSB0` |
| Left ID | `2` |
| Right ID | `0x20` |
| Baud rate | `115200` |

The publisher waits one second after binding to allow the ZMQ subscriptions to
settle. PUB/SUB messages sent before the connection is established are not
replayed.

## Publisher modes

```bash
# Keep both hands at zero
python examples/zmq_teleop/zmq_publisher.py --demo zero --duration 10

# Interactive per-joint commands
python examples/zmq_teleop/zmq_publisher.py --demo keyboard --step 0.05

# Test joints one at a time
python examples/zmq_teleop/zmq_publisher.py --demo test

# Open/close sequence
python examples/zmq_teleop/zmq_publisher.py --demo grasp

# Sinusoidal demonstration
python examples/zmq_teleop/zmq_publisher.py --demo sine --duration 10
```

Use `--port-left` and `--port-right` as shortcuts for wildcard TCP endpoints.
Run `python examples/zmq_teleop/zmq_publisher.py --help` for every option.

## Message schema

Each endpoint carries one hand and must contain exactly 10 finite positions in
radians:

```json
{
  "timestamp": 1770000000.125,
  "qpos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "joint_names": [
    "left_thumb_proximal",
    "left_thumb_intermediate",
    "left_index_proximal",
    "left_middle_proximal",
    "left_ring_proximal",
    "left_pinky_proximal",
    "left_index_intermediate",
    "left_middle_intermediate",
    "left_ring_intermediate",
    "left_pinky_intermediate"
  ]
}
```

The right endpoint uses the same suffix order with the `right_` prefix. The C++
example uses array order for motor mapping; names are included for diagnostics.
Malformed, non-finite, short, or long `qpos` arrays are rejected without
changing that hand's target.

## Remote publisher

On the robot computer, connect the subscriber to the publisher computer:

```bash
./build/zmq_teleop/casia_zmq_teleop \
  tcp://192.168.1.20:5555 tcp://192.168.1.20:5556 \
  /dev/ttyUSB0 2 0x20 115200
```

On `192.168.1.20`, bind the publisher to both interfaces as shown in the local
example. Open both TCP ports in the firewall. Do not use
`tcp://0.0.0.0:PORT` as a subscriber connect address; `0.0.0.0` is for bind.

## Troubleshooting

- `Package libzmq was not found`: install `libzmq3-dev` and ensure
  `pkg-config --modversion libzmq` succeeds.
- `can not find port`: use the real device path. Both `/dev/ttyUSB<N>` and
  `/dev/ttyCH341USB<N>` are supported; rebuild after updating the SDK. Check
  host-side udev permissions and Docker `--device` mapping if applicable.
- initialization reports a missing hand: verify both IDs and baud rate; this
  dual-hand SDK intentionally refuses to start with only one hand online.
- commands are received but motion is unexpected: stop immediately and verify
  the 10-element motor order above.
- no ZMQ messages arrive: start the publisher first and verify that the
  subscriber connects to the publisher host rather than binding the same port.
