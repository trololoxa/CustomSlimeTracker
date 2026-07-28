# 0023gj SlimeVR Connect Trackers session-restart hardening

## Exact predecessor

- User-supplied `Tracker.zip`
- SHA-256: `56942b020205decf25f4198272b69327e914eb8133a4e9bf9a568e9d43ec6017`
- The archive already contains the `0023gi` handshake/build-date changes and is the only predecessor used for this hotfix.

## Confirmed defect before the fix

### Exact scenario

1. The tracker is already connected to Wi-Fi and has an established SlimeVR UDP session (`serverFound=true`).
2. The user opens the server's **Connect trackers** flow and connects that tracker over USB.
3. The server sends `SET WIFI` or `SET BWIFI`; the supplied credentials may be identical to the credentials already in use.
4. The firmware persists the credentials, resets/reconfigures Wi-Fi, prints the exact success acknowledgement, and invokes the local equivalent of `slime start`.
5. `slime start` only calls `SlimeVROutputRuntime::configure()`. When the network-facing SlimeVR configuration is unchanged, `configure()` preserves the existing UDP session.
6. If the Wi-Fi station reconnects quickly enough that `SlimeVROutputRuntime::update()` never observes a disconnected state, `serverFound` remains true. No fresh discovery handshake is sent, the server does not see a new onboarding discovery event, and a fresh `SensorInfo` registration/acknowledgement cycle does not occur.

### What breaks

Serial provisioning can report success (`wifi state: 3 -> 5`) while the tracker fails the network phase of **Connect trackers**. The failure is deterministic for the already-connected/same-credentials edge case and cannot be correctly attributed only to firewall or broadcast routing.

### Severity and risk

- Severity: **high onboarding interoperability defect**.
- Tracking hot path: unaffected while the defect is dormant.
- User impact: an otherwise healthy tracker may require a power cycle, manual `slime reconnect`, or an actual Wi-Fi disconnect before it appears as newly connected in the wizard.
- Data/storage risk in the predecessor: the live network config and Wi-Fi state were changed before NVS save completed, so a failed persistent write could leave runtime credentials changed despite an error acknowledgement.

### Exact source

- `src/serial/tracker_slimevr_serial_compat_commands.cpp`
  - `setWifiCredentials()` mutated live config before the commit point.
  - It reset Wi-Fi and recursively dispatched `slime start` through the CLI parser.
- `src/serial/tracker_slimevr_commands.cpp`
  - `slime start` configured the runtime but deliberately did not restart an existing session.
- `src/runtime/slimevr_output_runtime_impl.inc`
  - `configure()` resets the connection only when its own network-facing runtime fields change.
  - SSID/password are owned by `TrackerWifiManager`, so resubmitting the same credentials can leave the SlimeVR runtime configuration unchanged.
  - `restart()` already provides the correct bounded operation: stop UDP, clear endpoint/session/feature/SensorInfo state, reset packet numbering, remain enabled, and return to discovery after Wi-Fi is available.

### Why predecessor tests missed it

`0023gi` tested healthy serial status, discovery packet number zero, firmware/build-date metadata, and packet-writer sequencing. It did not model:

- an already-established server session;
- Wi-Fi remaining logically connected while credentials are resubmitted;
- explicit session invalidation after serial provisioning;
- a second discovery response from the same server;
- a fresh `SensorInfo` send and acknowledgement after provisioning.

## Official-code audit

Audit date: 27 July 2026.

### Server onboarding and serial side

The public setup flow asks the user for Wi-Fi credentials, sends them to the USB-connected tracker, waits for the tracker to connect to Wi-Fi, and then waits for the tracker to appear through the normal network protocol. Current official tracker firmware supports both plaintext `SET WIFI` and base64 `SET BWIFI`; both return the exact success text `New wifi credentials set, reconnecting`. `GET INFO` reports tracker health separately from `wifi state`.

The current server repository still has open PR `#1768` to move provisioning to base64 `BWIFI`, so supporting both commands remains required for released and newer server builds.

### Official tracker behavior

Relevant official files:

- [`SlimeVR-Tracker-ESP/src/serial/serialcommands.cpp`](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/main/src/serial/serialcommands.cpp)
- [`SlimeVR-Tracker-ESP/src/network/connection.cpp`](https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/main/src/network/connection.cpp)

The official serial command handler:

- decodes and validates the supplied credentials;
- hands them to the Wi-Fi subsystem;
- prints the exact success acknowledgement immediately;
- exposes current tracker/Wi-Fi state through a separate `GET INFO` query.

The official network path repeatedly sends discovery while no server is selected, accepts only the special server response `0x03 + "Hey OVR =D 5"`, then performs the normal session lifecycle.

### Server UDP behavior

Relevant official files:

- [`TrackersUDPServer.kt`](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/TrackersUDPServer.kt)
- [`UDPProtocolParser.kt`](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPProtocolParser.kt)
- [`UDPDevice.kt`](https://github.com/SlimeVR/SlimeVR-Server/blob/main/server/core/src/main/java/dev/slimevr/tracking/trackers/udp/UDPDevice.kt)

The server:

1. Parses the normal packet ID and packet number.
2. Accepts packet number zero regardless of prior sequence state.
3. On a handshake, first looks up an existing UDP device by MAC.
4. For an existing MAC, updates the address/metadata and resets `lastPacketNumber` instead of creating a duplicate device.
5. Sends the special handshake response.
6. On a subsequent packet 15, creates or updates the sensor and sends the short `SensorInfo` acknowledgement.

Therefore a fresh handshake from an already-known tracker is explicitly supported and is the correct way to make an already-connected device pass the network phase of **Connect trackers**. The firmware must not rely on the physical Wi-Fi link visibly dropping first.

## Implementation

### Shared typed runtime apply path

`tracker_slimevr_commands.hpp/.cpp` now exposes:

```cpp
enum class TrackerSlimeVRRuntimeApplyMode : uint8_t {
    PreserveSession,
    RestartSession,
};

bool trackerSerialApplySlimeVRRuntimeConfig(...);
```

The helper owns all common runtime configuration work:

- sanitize the network config;
- stop conflicting local serial streaming;
- configure `SlimeVROutputRuntime` from the current network/product config;
- optionally call `restart()` when a new session is required.

This removes recursive CLI dispatch from the compatibility layer and prevents future drift between `slime start`, `slime reconnect`, and server provisioning.

Mode ownership is explicit:

- `slime start` -> `PreserveSession`;
- `slime reconnect/restart` -> `RestartSession`;
- successful `SET WIFI`/`SET BWIFI` -> `RestartSession`.

### Transactional credential update

`setWifiCredentials()` now:

1. Requires the config, config store, Wi-Fi manager, and SlimeVR runtime.
2. Builds a separate `TrackerNetworkConfig candidate`.
3. Sanitizes and saves the candidate.
4. Returns failure without touching live state if the NVS commit fails.
5. Activates the committed candidate.
6. Resets/reconfigures Wi-Fi.
7. Explicitly restarts the SlimeVR session.

The order is enforced by policy as:

```text
commit candidate
-> activate live config
-> reset/reconfigure Wi-Fi
-> restart SlimeVR UDP session
```

No delay, polling loop, heap allocation, schema change, or new persistent field is introduced.

### Session restart semantics

The existing `SlimeVROutputRuntime::restart()` is reused. It clears:

- selected server endpoint;
- `serverFound`;
- discovery timers;
- incoming-session timestamp;
- negotiated server feature flags;
- sent/acknowledged `SensorInfo` state;
- packet writer sequence;
- send-failure/reopen transient state.

Counters are preserved for diagnostics. On the next normal update with Wi-Fi connected, UDP is reopened and a packet-number-zero discovery handshake is sent. After the server response, a fresh `SensorInfo` is sent and acknowledged.

## Tests and policy gates

### Runtime regression

`tests/native/test_slimevr_output_runtime.cpp` now constructs the exact missed scenario:

- Wi-Fi remains connected;
- an initial discovery session is established;
- `SensorInfo` is acknowledged;
- the runtime is explicitly restarted as provisioning requires;
- the old endpoint/session and feature state are cleared;
- a new zero-numbered discovery is emitted without a Wi-Fi-disconnected observation;
- the same server is accepted again;
- a second `SensorInfo` is sent and acknowledged.

### Policy

`tools/test_slimevr_connect_trackers_0023gj_policy.py` enforces:

- typed `PreserveSession`/`RestartSession` ownership;
- one shared runtime apply helper;
- no recursive CLI dispatch from serial compatibility;
- transactional NVS-before-live ordering;
- explicit Wi-Fi reset and SlimeVR session restart;
- non-blocking provisioning;
- the complete already-connected runtime regression;
- native and Production-profile compilation of affected serial units.

The predecessor `0023gh` policy was updated only where its old implementation-specific assertion conflicted with the now-stronger transactional contract. All original protocol values and exact acknowledgement strings remain locked.

## Validation performed

Passed in the audited tree:

- `git diff --check`;
- all 60 host-compilable project/compile-only/Arduino translation units with the canonical native warning flags;
- all 41 standalone native test executables, including the new already-connected runtime regression;
- `0023gh`, `0023gi`, and `0023gj` SlimeVR provisioning/session policies;
- all calibration policies through `0023gg`, calibration integration, and magnetic heading reliability;
- source-filter, profile-matrix, documentation, build-identity, `check_all`, aggregation, standalone-runner, SlimeVR session, storage, stack, and autonomy policies;
- 60-second replay, replay comparison, MAGR replay, and 600-second baseline replay;
- focused `SlimeVROutputRuntime` test under AddressSanitizer and UndefinedBehaviorSanitizer;
- `git apply --check`, real `git apply`, GNU `patch --dry-run -p1`, and real `patch -p1`;
- byte-and-mode tree equality for both independently patched trees against the audited tree, excluding only generated build/cache directories.

The monolithic sequential `check_all.py` invocation exceeded the execution environment's 15-minute command limit while compiling/running already-passing components. Its native and tool-smoke constituents were therefore completed separately with the same canonical flags and commands; no constituent failure was observed.

PlatformIO is not installed in the audit environment. ESP32-C3 firmware builds were not claimed or simulated. The user-side mandatory build gate remains:

```bash
python3 tools/check_all.py --clean --require-pio
```

## Runtime and compatibility impact

Unchanged:

- AHRS, FIFO, ODR, timestamps, tracking rate and packet payloads;
- persistent schema and NVS namespace;
- calibration data and candidate formats;
- SlimeVR protocol version 22;
- normal boot/session behavior;
- normal `slime start` non-disruptive behavior.

Changed only after successful server-style credential provisioning:

- the current UDP session is deliberately invalidated;
- discovery and `SensorInfo` registration are repeated;
- the tracker may briefly disappear/reappear in the server, which is required onboarding behavior.

## Hardware acceptance

Use a tracker that is already visible and actively sending rotations to the server.

1. Record:

```text
GET INFO
slime status
```

Expected before provisioning:

```text
status: 0
wifi state: 5
server_found=yes
sensor_info_sync_state=acknowledged
```

2. Open **Connect trackers**, connect the same tracker over USB, and submit the same SSID/password already stored on it.

3. Expected serial flow:

```text
CMD SET WIFI OK: New wifi credentials set, reconnecting
```

or:

```text
CMD SET BWIFI OK: New wifi credentials set, reconnecting
```

`GET INFO` should show the server-credential attempt and then success (`wifi state: 3 -> 5`; a very fast reconnect may make the intermediate sample easy to miss).

4. Without rebooting or power-cycling, verify:

```text
slime status
```

Expected lifecycle:

```text
server_found=no
-> discovery handshake sent
-> server_found=yes
-> sensor_info_sync_state=waiting_for_ack
-> sensor_info_sync_state=acknowledged
```

Useful counter deltas:

```text
handshakes_sent +1 or more
sensor_info_sent +1
sensor_info_ack_received +1
discovery_responses +1
```

5. Confirm the tracker appears/completes in **Connect trackers**, resumes normal rotation delivery, and has no new:

```text
wifi disconnect loop
udp_begin_failures
send_failures
unknown_packets_received
FIFO overrun/full
tracking recovery
```

A remaining failure after this complete lifecycle is then legitimately outside this firmware defect boundary: firewall/broadcast routing, AP isolation, separate subnets, VPN routing, competing SlimeVR servers, or host serial access.
