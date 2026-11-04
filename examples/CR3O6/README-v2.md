# CR3/O6 Customer API v2 prediction client

`deploy_cr3_o6_customer_v2.py` is a separate client for
`customer-integration-kit-20261026`. It uploads three real 640×480 RGB images
and the measured 12-value CR3/O6 state, then prints the full 16×12 action
prediction. The default task is the SDK's canonical three-finger task.

Prediction-only mode is the default. Real motion requires both
`--execute-live` and `--confirm-motion`; it homes the arm and all six O6
channels at startup, keeps ServoJ open across inference chunks, and attempts
to return to the configured initial poses on exit. The v2 kit reports public
inference latency of about 1.10–4.50 seconds against a nominal 0.8-second
action horizon, and does not validate continuous real-time execution. Actions
are scheduled against their observation timestamps; expired steps are
discarded, never replayed. With the currently documented latency, a whole
chunk may expire before arriving, resulting in no motion. Confirm control and
stop behavior in a cleared workspace with an operator present.

The default v2 profile is `config-v2.toml`; it uses the new v2 service URL and
reuses the existing private token file without copying or printing its
contents. Adjust local robot/SDK paths there if this checkout is on another
machine. No integration-kit files are modified.

Run one observation/prediction request without motion:

```bash
./examples/CR3O6/run_cr3_o6_customer_v2.sh
```

Keep requesting fresh observations and predictions until Ctrl-C, without motion:

```bash
./examples/CR3O6/run_cr3_o6_customer_v2.sh --continuous
```

Explicitly enable continuous live execution:

```bash
./examples/CR3O6/run_cr3_o6_customer_v2.sh --execute-live --confirm-motion
```

Select the SDK's two-finger preset with `--task two_finger`.
The camera adapter refuses resizing in this v2 client. The program also stops
if measured O6 state channels 10 or 11 are nonzero rather than modifying the
observation to satisfy the API validator. Hand action channels 6–9 control the
thumb, index and middle fingers; channels 10–11 are the fixed-zero channels.

The launcher chooses an installed Python environment and exposes its
`libpython` directory for the NRC binding. Install the v2 kit requirements
before running, in addition to the existing camera and robot SDK dependencies.
