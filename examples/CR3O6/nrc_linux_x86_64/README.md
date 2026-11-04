# CR3 NRC vendor runtime

Place the vendor-provided files here, or set another directory as
`[robot].nrc_sdk_dir` in `examples/CR3O6/config.toml`:

```text
nrc_linux_x86_64/
├── nrc_interface.py
└── _nrc_host.so
```

These files are generated/provided by the Dobot/NRC SDK. They are not
recreated by this repository and may have platform, CPython, `libpython`, and
vendor redistribution restrictions. The local wrapper checks for both files
before connecting to the robot.
