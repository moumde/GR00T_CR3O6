# CR3/O6 sequential segmented inference client

This local client uses the redeploy session and SDK, but runs a repeated,
sequential chunk loop. It homes the arm and hand at startup, captures an
observation, requests one 16-step action chunk, executes it, then holds the
resulting pose while capturing and sending the next observation. It does not
home between chunks. Ctrl-C or the 120-second trial deadline ends the loop;
the client then attempts to return the arm and hand to their configured home
poses. Controller faults are not cleared and failed requests are not retried.

Run from the repository root:

```bash
./examples/CR3O6/run_cr3_o6_stationary_auto.sh
```

Each action chunk covers 16 steps at 50 ms (0.8 seconds). The client starts
that chunk's timing after its response arrives. It holds the current arm pose
while waiting for the next prediction, so network delay creates a pause between
segments. Recent client inference round trips have been several seconds; this
is segmented stop-and-wait motion, not a smooth 20 Hz stream. The local client
does not enforce the redeploy manifest's one-request/one-segment fields; the
service may still enforce its own quota.

Each run writes a uniquely timestamped JSON report and `.log` file into
`examples/CR3O6/stationary-reports/`. The log contains stage timings, each full
returned 16×12 action chunk, actuator command samples, and feedback. It does
not save camera images or the token.

The API URL and expiry are checked against the redeploy manifest and live
metadata. The active URL is
`https://pearl-elementary-earth-hotel.trycloudflare.com`; the session expires
at `2026-10-30T06:50:49.852796+00:00`.

The client uses the local proxy-enabled SDK copy with HTTPS certificate
verification and redirect refusal. Logs include network exception class and
OS error number when available, without printing proxy credentials.
