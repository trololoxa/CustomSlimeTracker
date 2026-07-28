# 0023gi SlimeVR Connect Trackers handshake and build-date hardening

## Exact predecessor

- `Tracker_0023gh_slimevr_wifi_provisioning_compat_hardening.zip`
- SHA-256: `659bae925c23f1749a729213e61196ab81444831ef4e619656b7f3633a09ded2`

## Protocol audit

The server-side Connect Trackers flow has two distinct acceptance phases. It first provisions credentials over serial (`SET BWIFI` in current server builds), accepts the exact success acknowledgement, and polls `GET INFO` until the firmware reports Wi-Fi `Success=5` or `Failed=4`. Wi-Fi success alone is not completion: the wizard then waits for the tracker to appear in the server through the normal UDP discovery/session path.

The official tracker keeps the general tracker-health `status` field at `0` for a healthy sensor, independently of Wi-Fi progress. Its current `Connection::sendTrackerDiscovery()` forces the UDP discovery packet number to `0`; `searchForServer()` repeats discovery about once per second while disconnected. The packet contains board/IMU/MCU identifiers, protocol version, a length-prefixed firmware string, MAC address, tracker type and optional vendor/product/update strings. The tracker accepts the server only after receiving the special reply consisting of byte `Handshake` followed immediately by `Hey OVR =D 5`.

The server UDP parser reads the ordinary four-byte packet type and eight-byte packet number, parses the handshake metadata including protocol version and the length-prefixed firmware string, creates or updates the UDP device, and replies with that special handshake response. Packet sequence ordering is enforced once a connection already exists. For initial discovery, forcing zero is therefore primarily exact official-protocol compatibility and guarantees discovery retries do not consume the later session sequence; it should not be presented as the only possible cause of an onboarding timeout.

The current public server work also shows the Connect Trackers serial side moving toward base64 `BWIFI`. Public setup documentation requires the exact successful acknowledgement and then expects the tracker to appear through normal network discovery. If serial provisioning succeeds but UDP discovery responses remain zero, firewall, network profile, AP/client isolation, VPN routing or separate subnets remain external blockers.

## Confirmed defects in 0023gh

1. `GET INFO status` was incorrectly used as a network progress code: disconnected/connected/server-found yielded `0/1/2`. Official healthy output uses `status: 0`; Wi-Fi progress belongs only to `wifi state`.
2. The first local discovery handshake happened to use packet number zero, but retries consumed `1,2,3...`. Official discovery retries always force zero. This was a confirmed protocol divergence and unnecessarily consumed the future session packet sequence. The server may still parse an initial handshake without an established sequence context, so this mismatch was a plausible interoperability risk rather than proof of the sole timeout cause.
3. Serial compatibility hardcoded `firmware: track-fw`, and the UDP handshake exposed only the feature version. No build date reached the server.

## Implementation

- Healthy initialized IMU -> serial `status: 0`; sensor initialization failure remains `3`. Wi-Fi and server state are no longer consulted by `statusCode`.
- `SlimeVRPacketWriter::writeHandshake()` writes type plus an explicit 64-bit zero and does not advance `nextPacketNumber_`. Normal session packets retain monotonic numbering.
- PlatformIO-generated build identity now includes `TRACKER_BUILD_DATE_UTC`, compact date, and `TRACKER_BUILD_SLIMEVR_FIRMWARE_VERSION`.
- Build date comes from `SOURCE_DATE_EPOCH` when supplied, otherwise current UTC date. Invalid epochs fail the build instead of silently inventing metadata.
- UDP handshake and serial `GET INFO`/`GET TEST` use `<feature-version>+build.YYYYMMDD`. `GET INFO` also prints the actual Git/worktree identity and `Build date: YYYY-MM-DD`; `version` exposes both values.

## Runtime impact

No Wi-Fi/AHRS/FIFO/tracking loop was added. Handshake serialization runs only at discovery cadence (normally once per second before a session). The status/build strings are emitted only on serial queries or discovery. No heap, queue, schema, NVS or packet-rate changes were introduced.

## Acceptance

After flashing, Connect Trackers should show the serial transition `wifi state: 3 -> 5`, then the tracker should appear through UDP discovery. `GET INFO` should report `status: 0`, the assigned IP/MAC, dated firmware version, Git identity and build date. If Wi-Fi reaches 5 but the tracker still does not appear, the remaining likely boundary is host/router UDP broadcast reception (firewall, public network profile, AP isolation, guest network, VPN, multiple server instances), not serial provisioning.
