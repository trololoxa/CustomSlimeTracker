# Network layer

Reserved for a future real Wi-Fi/UDP transport.

Do not add placeholder SlimeVR output here. The expected split is:

- `network/`: Wi-Fi connection and UDP transport primitives.
- `output/` or `runtime/`: packet formatting/output runtime that consumes a prepared quaternion/status snapshot.
- `serial/`: CLI commands that enable/configure the backend only after it exists.

Until then, `output mode binary` and `output mode slimevr` must remain `NOT_IMPLEMENTED`.
