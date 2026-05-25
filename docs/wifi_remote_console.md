# Wi-Fi remote console

The firmware can expose the normal CLI over a small TCP server for cable-free
setup and calibration. This is not a separate command set: the remote console
feeds the same `TrackerSerialCommandInterface` dispatcher that USB Serial uses.

Defaults:

- Debug: enabled.
- Production: enabled.
- Slim: disabled and excluded from the source filter.
- Port: `TRACKER_REMOTE_CONSOLE_PORT`, default `7777`.
- Per-loop input budget: `TRACKER_REMOTE_CONSOLE_BYTES_PER_LOOP`, default `32`
  in Debug and `16` in Production.

Connect from a PC on the same Wi-Fi network:

```bash
nc <tracker-ip> 7777
```

or:

```bash
telnet <tracker-ip> 7777
```

Typical wireless calibration flow:

```text
setup status
setup calibration
remote off
```

## Runtime control

The remote console is intentionally runtime-disableable. After calibration, run:

```text
remote off
```

This closes the current TCP client and stops the TCP server for the current
boot. After that the runtime no longer accepts/polls TCP console clients, so it
does not add normal-loop or Wi-Fi server work. It does not change NVS. USB Serial
can re-enable it for the current boot with:

```text
remote on
```

Status:

```text
remote status
```

prints whether the feature is compiled, enabled, listening, connected, the port,
and simple client/byte counters.

## Notes

- Only one TCP client is accepted at a time. Extra clients receive a busy message
  and are closed.
- The console is intentionally unauthenticated. Use it only on a trusted private
  network or compile it out with `-DTRACKER_ENABLE_WIFI_REMOTE_CONSOLE=0`.
- Blocking setup/calibration commands read from the same active TCP stream. The
  app does not poll the remote-console parser from the blocking calibration
  service hook, so prompt input is not stolen recursively.
