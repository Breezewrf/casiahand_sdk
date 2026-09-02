# casiahand-sdk

Self-contained Linux Python package for a dual CASIA Hand-M. It vendors the
CASIA C++ sources and their serial dependency, builds one pybind11 extension,
and provides a non-blocking runtime designed for RoboJuDo Recorder.

The data model deliberately keeps commands and feedback separate:

- `joint_position_commands`: the clipped 20-joint command accepted by the
  native SDK worker (Recorder `action`).
- `joint_positions`: the latest independently sampled CASIA motor positions
  (Recorder `state`).

Measured positions are never replaced by commands. Hardware freshness advances
only when a new serial sample is consumed; a cached value does not keep
`joint_state_fresh` true after feedback stops.

## Repository layout

```text
casiahand_sdk/
├── CMakeLists.txt                 # self-contained native build
├── pyproject.toml                 # pip/scikit-build package
├── sdk/                           # patched CASIA C++ SDK + serial dependency
├── src/casiahand_sdk/             # public Python API and async runtime
├── src/casia_hand_cpp/            # legacy import compatibility
├── integrations/robojudo/         # RoboJuDo adapter and config addition
├── examples/zmq_teleop/           # standalone dual-stream ZMQ teleop
└── tests/                          # hardware-free runtime tests
```

No separately installed CASIA library, `CASIA_HAND_SDK_DIR`, `ZKGJ_LIBS`, or
`LD_LIBRARY_PATH` is required. The native SDK and serial implementation are
statically linked into the Python extension.

## Requirements

- Linux x86-64 or a Linux target that can compile the included C++ sources
- Python 3.10 or newer (RoboJuDo uses Python 3.11)
- CMake 3.16+
- a C++17 compiler
- Python development headers
- Boost headers (`boost::optional` is used by the vendor SDK)

On Ubuntu:

```bash
sudo apt-get install build-essential cmake python3-dev libboost-dev
```

## Use as a RoboJuDo submodule

From the RoboJuDo-Plus repository root:

```bash
git submodule add <casiahand-sdk-repository-url> packages/casiahand_sdk
python -m pip install -e packages/casiahand_sdk
```

For `submodule_install.py`, add this entry to `submodule_cfg.yaml`:

```yaml
casiahand_sdk:
  install: false
  path: packages/casiahand_sdk
  local: true
```

Then installation is also available through:

```bash
python submodule_install.py casiahand_sdk
```

The editable install still compiles the native extension. Reinstall after
changing any file under `sdk/` or `src/py_binding.cpp`.

## RoboJuDo integration

The submodule contains two integration files:

- `integrations/robojudo/controller/casia_hand_runtime.py`
- `integrations/robojudo/ctrl_cfgs_addition.py`

Copy the controller adapter into RoboJuDo:

```bash
cp packages/casiahand_sdk/integrations/robojudo/controller/casia_hand_runtime.py \
  robojudo/controller/casia_hand_runtime.py
```

Add `CasiaHandCfg` from `ctrl_cfgs_addition.py` to
`robojudo/controller/ctrl_cfgs.py`. A RoboJuDo `CasiaHandCfg` is structurally
compatible with the runtime: it only needs the documented attributes, so no
conversion is required.

```python
from robojudo.controller.casia_hand_runtime import CasiaHandRuntime
from robojudo.controller.ctrl_cfgs import CasiaHandCfg

runtime = CasiaHandRuntime(
    CasiaHandCfg(
        left_hand_id=2,
        right_hand_id=0x20,
        baudrate=115200,
        port_name="/dev/ttyUSB0",
    )
)
```

The controller or pipeline that owns the runtime must call `close()` during
shutdown.

## Runtime API

```python
import numpy as np

from casiahand_sdk import CasiaHandConfig, CasiaHandRuntime

runtime = CasiaHandRuntime(CasiaHandConfig(port_name="/dev/ttyUSB0"))
try:
    runtime.set_takeover_enabled(True)
    accepted = runtime.set_joint_commands(
        left_joint_positions=np.zeros(10),
        right_joint_positions=np.zeros(10),
        source_timestamp_ns=None,
        frame_id=1,
    )
    data = runtime.get_data()
finally:
    runtime.close()
```

`set_joint_commands()` is non-blocking and returns the 20 clipped command
values. The worker keeps only the newest pending dual-hand frame. Commands are
discarded while takeover is disabled.

`get_data()` returns consistent copies:

```python
{
    "joint_names": [...],                 # 20 names, left then right
    "joint_positions": measured,          # float32[20], hardware feedback
    "joint_position_commands": applied,   # float32[20], native SDK command
    "left_fresh": bool,
    "right_fresh": bool,
    "joint_state_fresh": bool,
    "fresh": bool,
    "age_s": float | None,
    "enabled": bool,
    "applied_source_timestamp_ns": int | None,
    "applied_frame_id": int | None,
}
```

`fresh` requires takeover to be enabled, a command applied within
`command_timeout_s`, and a new measured sample within
`joint_state_timeout_s`. Source timestamps are retained only for correlation;
freshness uses local monotonic time.

## Joint order and limits

Each call supplies 10 left and 10 right values in radians. Both hands use this
motor order:

1. `thumb_proximal`
2. `thumb_intermediate`
3. `index_proximal`
4. `middle_proximal`
5. `ring_proximal`
6. `pinky_proximal`
7. `index_intermediate`
8. `middle_intermediate`
9. `ring_intermediate`
10. `pinky_intermediate`

The full names are prefixed with `left_` and `right_`. Commands are clipped to
`[0, 1.57]` rad except `thumb_intermediate`, whose upper limit is
`1.57 * 7 / 9` rad. This matches the scaling used by the CASIA SDK. Retargeting
code that represents the second thumb joint as negative must convert it to the
non-negative hardware convention before sending it; the existing CASIA teleop
path uses `abs()` for this conversion.

## Native API

Low-level use is available when the application owns its own I/O thread:

```python
from casiahand_sdk import CasiaHand

hand = CasiaHand(2, 0x20, 115200, "/dev/ttyUSB0")
if not hand.init():
    raise RuntimeError("CASIA initialization failed")
try:
    applied = hand.set_joint_positions([0.0] * 20)
    measured = hand.try_get_joint_positions()  # list[20] or None
finally:
    hand.close()
```

All methods that may enter C++ release the Python GIL. Do not call one
`CasiaHand` instance concurrently from multiple application threads; use
`CasiaHandRuntime` for normal RoboJuDo operation.

## Hardware setup

Identify the serial device and grant access before starting RoboJuDo:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. A temporary development
alternative is `sudo chmod 666 /dev/ttyUSB0`, but persistent udev/group rules
are preferable.

Default protocol settings are:

| Setting | Default |
|---|---:|
| Left device ID | `2` |
| Right device ID | `0x20` |
| Baud rate | `115200` |
| Port | `/dev/ttyUSB0` |
| Command power per hand | first 6 motors `15`, last 4 motors `3` |
| Synchronized speed | `0.6` |

Confirm the IDs and baud rate against the actual hand configuration before
enabling takeover.

## Testing

Build a wheel from a clean checkout:

```bash
python -m pip wheel . --no-deps --wheel-dir wheelhouse
```

Install and run hardware-free tests:

```bash
python -m pip install -e ".[test]"
python -m unittest discover -s tests -v
```

The tests use a fake native hand and cover:

- separation of measured state and applied action
- atomic latest-command-wins behavior
- physical command clipping
- takeover gating
- state timeout when no new hardware sample arrives
- invalid command rejection

A real-hardware smoke test should additionally verify joint direction and order
at low speed with the hand unloaded before recording data.

## Standalone ZMQ teleop

The original non-RoboJuDo teleop is retained under
[`examples/zmq_teleop`](examples/zmq_teleop). It builds against the same
vendored SDK used by the Python extension and accepts separate left/right ZMQ
JSON streams. Its physical-hand defaults match dex_teleop: left port `5555`,
right port `5556` (`5560/5561` remain reserved for simulation). See its README
for dependencies, exact message schema, build and safe startup instructions.

## Vendor SDK changes included here

The vendored SDK is minimally adjusted for Python/runtime safety:

- state queue reads consume a new sample instead of repeatedly reporting a
  cached sample
- state is published only after a successful serial measurement
- failed initialization can be destroyed safely
- the control-thread flag is atomic and state structures are initialized
- command angles are clamped in radians to the SDK's physical scaling limits
- power commands are clamped to the per-motor physical scaling limits
- initialization requires both configured hands to be online
- copying the owning `CasiaHandM` object is disabled

These changes are necessary for trustworthy Recorder freshness semantics.

## Troubleshooting

`ModuleNotFoundError: casiahand_sdk`

: Install the submodule into the same Python interpreter that launches
  RoboJuDo: `python -m pip install -e packages/casiahand_sdk`.

Build cannot find `pybind11` or `scikit-build-core`

: Allow pip to install the PEP 517 build requirements, or preinstall them with
  `python -m pip install scikit-build-core pybind11`.

`can not find port`

: Check `port_name`, cable presence, device enumeration, and dialout access.

Runtime starts but `joint_state_fresh` is false

: No new valid serial feedback has arrived within the configured timeout. Check
  device IDs, baud rate, serial permissions, and SDK timeout logs. Do not replace
  measured state with the last command to hide this condition.

## Third-party code

The serial library license is retained at
`sdk/third_party/serial/LICENSE`. The CASIA SDK sources remain subject to the
vendor's distribution terms; confirm those terms before publishing this
repository outside your organization.
