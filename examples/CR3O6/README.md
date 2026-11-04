# CR3/O6 Customer HTTPS client

This example connects the CR3/O6 robot to the Microsoft Customer HTTPS
inference API. It does not import the OpenPI project.

The `data_v21` contract is:

- video: `base_0_rgb`, `left_wrist_0_rgb`, `right_wrist_0_rgb`, each RGB `uint8` `(224, 224, 3)`;
- state: 6 CR3 joint angles in radians followed by 6 O6 values in `[0, 255]`;
- action: 6 absolute CR3 joint targets in radians followed by 6 absolute O6 targets;
- CR3 control rate: 20 Hz;
- returned action horizon: 16 steps.

The deployment script queries `get_modality_config()` from the cloud server
at startup; its returned modalities and horizon are the runtime contract.

## Dependencies

The robot-side process still needs the hardware vendor dependencies:

- the CR3 NRC binding (`nrc_interface.py` and `_nrc_host.so`), located by `config.toml`;
- the LeRobot RealSense backend and `pyrealsense2`;
- the LinkerHand SDK, whose local directory is set in `config.toml`;
- the Customer SDK dependencies listed in the integration kit.

The CR3/O6 Python wrappers are now in `cr3_o6_interface.py`,
`cr3_o6_camera.py`, and `linkerhand_compat.py`. The NRC `.so` is a
vendor-owned, platform-specific binary and must be supplied according to the
vendor's redistribution terms. It is not an OpenPI dependency.

## Microsoft Customer HTTPS client

`customer-integration-kit-20261019` provides the HTTPS/JSON client. It does
not use the GR00T ZMQ server. The service currently reports `mode=shadow` and
`robot_execution_enabled=false`; explicitly confirmed local execution does not
use those fields as a motor-control switch.

Edit `examples/CR3O6/config.toml` for the session URL, token **file path**,
robot IP, SDK paths, camera mapping, and request timeout. Relative paths are
resolved from the config file's directory. The token itself stays in its
separate private file. Use `--config PATH` to select another profile; do not
put `--execute-live` or `--confirm-motion` in a profile. The launcher only
selects a Python environment and exposes its `libpython` to the NRC binding.
No environment exports are needed for the standard session.

Prepare the local environment once:

```bash
uv pip install --python .venv/bin/python \
  -r examples/CR3O6/customer-integration-kit-20261019/configs/customer_client_requirements.txt
uv pip install --python .venv/bin/python 'pymodbus==3.5.1' pyserial
chmod 600 examples/CR3O6/customer-integration-kit-20261019/api-token
```

The configured service URL is currently
`https://webpage-talented-employees-usd.trycloudflare.com` on TCP 443. Its
expiry is session-specific and is checked from the live `get_modality_config()`
response at startup. The default camera mapping is `top=base_0_rgb`, `wrist=left_wrist_0_rgb`, and
`side=right_wrist_0_rgb`. Confirm this physical mapping with the integration
owner, or change the `[cameras]` section of `config.toml`.

The startup log includes the service URL, remote mode and expiry; inference
logs include the request ID. Never share the token file.

The O6 SDK reads all six physical finger positions. For this Customer API,
only the first three are used as measured state; channels 9–11 are sent as
zero, matching `data_v21` and the SDK contract. The first observation logs
both the unused physical values and the uploaded zero values.

### Full real-robot inference

The complete local execution loop is:

```bash
./examples/CR3O6/run_cr3_o6_customer.sh --execute-live --confirm-motion
```

This powers on the CR3 if needed, returns the O6 to `(157,95,175,0,0,0)` and
all six CR3 joints to zero, then repeatedly captures three camera images plus
measured CR3/O6 state, sends them to the Customer API, and executes the
returned 16-step action chunk. There is no chunk-count limit. Within each
chunk CR3 commands run at 20 Hz and O6 commands at 10 Hz, matching the
observed RS485 throughput. In live mode ServoJ opens once per session, stays
open throughout inference. After the first response, a controller thread sends
CR3/O6 targets while the Customer client, on its original thread, samples a
new observation and requests the next response. O6 observation reads and
motor writes are serialized on the shared RS485 bus; a late hand update may
be skipped to preserve arm timing. The next action is bridged from current
measured positions with per-step motion limits, rather than rejected merely
because the arm moved during inference. No remote `mode=shadow` flag gates this
explicitly confirmed local execution. If observation plus network inference
takes longer than the remaining 0.8-second action horizon, the arm still
holds its last target until fresh actions arrive; the client does not
extrapolate an unverified trajectory. Logs report this hold duration.
The client accepts a remote result up to 3 seconds after the corresponding
observation; older results
still stop execution without sending their actions. Press
`Ctrl-C` to stop: the client keeps the active ServoJ session open while
returning the arm to zero, then returns the six O6 joints to the initial pose,
closes ServoJ and disconnects. This avoids closing and immediately reopening
ServoJ at a moving pose. Like the ROS
driver, normal shutdown does not automatically power off the CR3 servo, even
if this client powered it on at startup; use the teach pendant to power it off
when needed. CR3 homing continues to stream the zero target until measured
joint feedback has stayed within 0.03 rad for five 50 ms samples, or fails
after a 5 s settling timeout.
This return is also attempted if interruption happens during startup O6 homing,
once the CR3 connection is established. If the arm cannot return safely, O6
homing is skipped rather than opening the hand while a load may be held.
If startup arm homing itself fails, cleanup does not immediately reopen ServoJ
and retry the same failed move; the arm may not be at zero and needs manual
inspection.
If the CR3 is already in servo ERROR state on exit, the client records the
state, skips further arm/hand motion and power-off, and leaves the controller
alarm intact for diagnosis. Check the first servo-state transition in the log
and the teach-pendant alarm before restarting; this client does not clear
faults automatically during shutdown or restart.
`--duration-s 60` can instead end a run after 60 seconds of inference-loop
time. A network/control error also attempts the return; if the CR3 cannot
return safely (for example its servo is stopped), the hand return is skipped
and the failure is logged. The arm's home position must have a clear path and
the workspace must be monitored.

The launcher selects a Python environment, adds its `libpython` directory for
the NRC binding, and forwards arguments to the Python client. Running it without
`--execute-live --confirm-motion` exits before connecting to any hardware.
`--duration-s 60` optionally stops the inference loop after 60 seconds; without
it, press `Ctrl-C` to end the session and attempt the return to the initial pose.
`--config PATH` chooses another deployment profile.
