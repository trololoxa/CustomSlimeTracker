# Firmware profile contract

The project has three compile-time profiles. The contract is enforced in
`src/build_config/profile_contract.hpp` and selected from `platformio.ini` with
`TRACKER_BUILD_PROFILE`.

## Contract layers

Use two layers of macros:

- `TRACKER_ENABLE_*` describes a concrete module or command family.
- `TRACKER_HAS_*` describes whether a subsystem exists in the current build.

New app/runtime glue should prefer `TRACKER_HAS_*`. Keep low-level module files
on `TRACKER_ENABLE_*` only when they are implementing that exact module.

Examples:

```cpp
#if TRACKER_HAS_SERIAL_CLI
    deps_.runtime.cli->poll(TRACKER_CLI_BYTES_PER_LOOP);
#endif

#if TRACKER_HAS_CALIBRATION_UI
    g_calIo.fifo = &g_lsmFifo;
#endif
```

Avoid open-coded combinations in app/runtime glue when a derived alias exists:

```cpp
// Prefer this:
#if TRACKER_HAS_STATIC_TEST_STATE

// Instead of repeating this everywhere:
#if TRACKER_ENABLE_STATIC_TEST || TRACKER_ENABLE_BOOT_HEARTBEAT
```

## Compile-time checks

`profile_contract.hpp` validates common invalid combinations, for example:

- CLI command families require `TRACKER_ENABLE_SERIAL_CLI=1`.
- `TRACKER_ENABLE_SERIAL_CLI=1` requires `TRACKER_ENABLE_SERIAL_CONSOLE=1`.
- full CLI requires basic CLI.
- Wi-Fi remote console requires the CLI dispatcher and a non-zero TCP port/input budget.
- machine log, serial stream and boot heartbeat require the serial console.
- runtime power diagnostics require the runtime test runner.
- full config print requires config commands.

These checks intentionally fail at compile time. They are cheaper than allowing a
profile to reach link time with missing symbols from a source-filtered module.

## Source-filter rule

When a `.cpp` file is excluded from a profile, every public symbol from that file
must be handled by one of these patterns:

1. the declaration is gated by the same profile/feature flag;
2. a compact fallback exists in a header or always-compiled `.cpp`;
3. the caller is gated by `TRACKER_HAS_*` and cannot be compiled in that profile.

Do not rely on the linker to clean up profile drift. If a profile cannot use a
translation unit, exclude it with `build_src_filter` and keep the caller side
explicitly guarded.

## Serial stream state

`TRACKER_HAS_SERIAL_STREAM_STATE` is intentionally broader than
`TRACKER_HAS_SERIAL_STREAM`. Debug and Production can still need the lightweight
`TrackerSerialStreamState` for CLI/config compatibility or boot heartbeat even
when the local serial stream backend is disabled. Slim has no console, CLI,
serial stream or boot heartbeat, so this state is not instantiated there.
