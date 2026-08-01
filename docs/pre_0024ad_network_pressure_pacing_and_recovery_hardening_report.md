# pre-0024ad network-pressure pacing and recovery hardening

## Scope and predecessor

This continuation patch is built strictly on `pre_0024ac_imu_hotpath_and_slack_admission_hardening`. It changes only SlimeVR UDP scheduling, pressure recovery, reconnect fallback traffic, diagnostics, tests and policy documentation. IMU ODR, FIFO ordering, hardware timestamps, AHRS equations, accel/magnetic correction cadence, quaternion output, the 100 Hz target and packet payload layouts are unchanged.

## Hardware evidence

A five-hour session with 13 trackers showed a healthy IMU/FIFO pipeline and strong Wi-Fi association but persistent local UDP TX pressure:

```text
wifi_disconnects=0
fifo_overrun=0
fifo_full=0
last_udp_send_error=12
TX pressure failures=28,883
local UDP rebinds=1,261
full session reopen escalations=815
TX backoff drops=55,888
```

`errno=12` is the ESP/lwIP `ENOMEM` condition produced when lower Wi-Fi TX buffers cannot accept another datagram. The old 0023gl burst gate treated recurring pressure density as socket damage. With 13 trackers, intermittent successful motion continued, yet repeated local rebind and full discovery added `Handshake`, `SensorInfo`, `FeatureFlags`, heartbeat and temporary separate packet traffic to an already pressured channel.

The same log showed that CPU/FIFO was not the primary fault: FIFO overrun/full stayed zero and rotation software age normally remained in the low-millisecond range. Therefore this patch does not trade tracking quality for reduced load and does not introduce sample dropping or catch-up approximation.

## Defect

The predecessor recovery policy escalated on either four consecutive failures or eight failures in 32 physical attempts, then escalated a second burst near a rebind to full discovery. This was appropriate for a persistently broken local socket, but not for multi-tracker contention where `ENOMEM` is transient and successful motion still passes.

The result was a positive feedback loop:

```text
transient TX pressure
-> socket rebind
-> full session reopen
-> discovery and capability negotiation traffic
-> temporary separate rotation/acceleration fallback
-> additional datagrams during pressure
-> another TX-pressure episode
```

During capability negotiation the fallback could send packet 17 rotation at 100 Hz plus packet 4 acceleration at 50 Hz. The firmware did not repay missed rotation deadlines, but the reconnect fallback and control traffic could make the server's rolling packet/TPS display rise above the nominal 100 Hz after a drop.

## Implementation

### Pressure state machine

The runtime now distinguishes:

```text
Healthy
TransientPressure
SustainedPressure
AwaitingPostRebindSuccess
FullReopenCooldown
```

Recovery reasons are also explicit: motion stall, non-pressure transport failure, stale server RX, rebind failure and post-rebind motion stall.

`ENOMEM`, `ENOBUFS`, `EAGAIN` and `EWOULDBLOCK` begin or extend a pressure episode. The exact 8-of-32 bitmap remains fixed-memory diagnostic evidence but no longer initiates recovery by itself.

### Bounded pacing

Pressure backoff is 10, 20, 40 and 80 ms. Stale motion, tap, telemetry and optional background control are suppressed during the active backoff. Discovery and request/response control remain available for a genuinely lost session.

Suppressed `SensorInfo` and `FeatureFlags` do not consume their retry interval or failure budget. They retry after the bounded backoff instead of being silently delayed for a full normal interval.

### Recovery by successful-motion age

A local socket rebind is requested only when:

- a pressure episode is active;
- fresh motion attempts are still being made and failing;
- no successful motion datagram has passed for 500 ms;
- the server RX association is still recent.

Intermittent pressure with continuing successful motion keeps the socket, endpoint, packet sequence, `SensorInfo` state and negotiated bundle capability intact.

After a successful local rebind, the runtime waits up to another 1000 ms for a successful motion datagram. Only failure across that post-rebind interval escalates to full discovery. Rebind and full-reopen cooldowns are 5 s and 10 s respectively. A pressure episode closes only after 1000 ms of stable successful motion.

All timestamp states use explicit validity flags; `millis()==0` cannot be confused with “never observed”.

### Reconnect traffic

Until `FeatureFlags` negotiation is either confirmed or explicitly unavailable, reconnect fallback sends packet 17 rotation only. Packet 4 acceleration is suppressed during that bounded negotiation window. Once bundle support is confirmed, packet 100 resumes; if negotiation is unavailable, the legacy 50 Hz acceleration fallback resumes.

This preserves pose delivery while preventing a temporary 150-datagram/s motion burst during the exact period in which TX pressure is most likely.

### Deterministic phase distribution

After the immediate first pose, each tracker derives a deterministic millisecond phase from its MAC address and aligns its unchanged 100 Hz deadline to that phase. Thirteen simultaneously started trackers therefore do not all target the same local 10 ms boundary. The implementation does not change frequency or repay missed deadlines with a burst.

### Diagnostics

Status/perf/motion/runtime-test output now exposes:

```text
tx_pressure_state
tx_recovery_reason
tx_pressure_episode_active/count/duration/max
tx_pressure_stable_resets
last_successful_motion_tx_age_ms
successful_motion_tx_streak
udp_rebind_suppressed_cooldown
udp_full_reopen_suppressed_cooldown
physical_datagrams_sent
motion_datagrams_sent
separate_rotation_datagrams_sent
separate_acceleration_datagrams_sent
background_control_datagrams_sent
critical_control_datagrams_sent
motion_packet_mode_transitions
bundle_to_separate_transitions
separate_to_bundle_transitions
acceleration_suppressed_during_negotiation
rotation_phase_offset_ms
```

## Safety and quality invariants

- No IMU samples are removed, reordered or approximated.
- Gyro integration and accel/magnetic correction cadence are unchanged.
- The target rotation rate remains 100 Hz.
- Missed deadlines still advance directly to the nearest future deadline; no catch-up packet burst is generated.
- A local rebind preserves the server endpoint and negotiated session.
- Full discovery remains fail-closed when local rebind cannot restore motion.
- Fixed-size scalar/bitmap state only; no heap allocation or unbounded retry queue.
- Optional background control yields during pressure, while discovery and critical request/response control remain possible.

## Verification

The focused native regression covers intermittent pressure without socket churn, stable episode closure, sustained 500 ms motion stall, session-preserving rebind, post-rebind 1000 ms escalation, stale-server bounded reopen, failed-rebind fail-closed handling, background-control backoff/retry, and reconnect rotation-only negotiation. Optimized and ASan/UBSan builds plus stack-usage gates are owned by `tools/test_pre_0024ad_network_pressure_policy.py`.

## Completed verification

- 63/63 Production, compile-only and Arduino translation units compiled with the project warning profile.
- 44/44 native executables linked and passed.
- Focused runtime regression passed in optimized and ASan/UBSan builds.
- GCC `-fstack-usage`: `update=48 B`, `sendPacket=48 B`, `serviceTxRecovery=8 B`, `rebindUdpPreservingSession=32 B`.
- Source-filter, profile-matrix, documentation, build-identity, SlimeVR session, calibration-integration and magnetic-heading reliability gates passed.
- 60-second, MAGR and 600-second replay gates passed; replay metrics remained unchanged.
- PlatformIO was unavailable in the authoring environment, so ESP32-C3 compilation remains an explicit hardware-side acceptance gate.

## Hardware acceptance

After flashing, run a 13-tracker startup, active-motion and long-session test. `tx_pressure_failures` may remain non-zero under contention, but healthy behavior requires pressure episodes to close without recurrent socket churn. Local rebinds should be rare, full reopens should remain near zero, bundle mode should remain stable, and physical motion datagrams should remain close to the unchanged 100 Hz target rather than producing a reconnect-driven 120–125 TPS burst.
