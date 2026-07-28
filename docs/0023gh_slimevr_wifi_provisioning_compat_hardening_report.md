# 0023gh SlimeVR Wi-Fi provisioning compatibility hardening

## Predecessor

- `Tracker_0023gg_magnetic_timestamp_and_setup_acceptance_hardening.zip`
- SHA-256: `92eaf1a6baa41ad970a6ed000ac7874a9d9d41276bd2d0b32d13fe0ae26c077b`

## Hardware-visible symptom

SlimeVR Server's automatic setup wizard printed `Unable to connect to wifi` even though the tracker later obtained an address and connected to the server.

## Corrected protocol finding

The `wifi state` number in the upstream serial compatibility response is not Arduino `WiFi.status()` / `WL_*`. Upstream SlimeVR firmware emits its own `WiFiReconnectionStatus`:

- `NotSetup=0`
- `SavedAttempt=1`
- `HardcodeAttempt=2`
- `ServerCredAttempt=3`
- `Failed=4`
- `Success=5`

Therefore the existing `Connected -> 5` mapping was correct. The actual compatibility defects were:

1. internal `Connecting` was always reported as `2` (`HardcodeAttempt`) and could not distinguish an ordinary saved-credential attempt (`1`) from credentials submitted by the server (`3`);
2. internal `Backoff` was reported as `3` instead of `4` (`Failed`);
3. the successful `SET WIFI` / `SET BWIFI` acknowledgement said `saved to NVS`, while the upstream parser-facing text is `New wifi credentials set, reconnecting`.

The earlier hypothesis that success must be Arduino `WL_CONNECTED=3` was incorrect and is explicitly not implemented by this patch.

## Implementation

`src/serial/tracker_slimevr_serial_compat_commands.cpp` now owns a named local `SlimeVrWifiReconnectionStatus` enum with all six protocol values, a one-byte serial-provisioning attempt latch, and a constexpr internal-state adaptor. Compile-time assertions lock the provisioning-critical mappings:

- ordinary saved `Connecting -> 1`
- server-submitted `Connecting -> 3`
- `Backoff -> 4`
- `Connected -> 5`

The two success acknowledgements now exactly match upstream wording.

The command still returns immediately after credentials are transactionally written and reconnect is requested. It does not wait for association or DHCP, matching upstream behavior and preserving serial responsiveness.

## Runtime and technical-debt audit

- no tracking hot-path change;
- no IMU, FIFO, AHRS, UDP or 60 Hz magnetic callback change;
- no heap allocation;
- no queue or persisted-state growth; the serial-compatibility build adds one byte of transient BSS to remember whether the current credentials came from the server;
- no schema, NVS layout or migration change;
- no blocking wait or retry loop in the serial command;
- named protocol values replace undocumented magic-number semantics;
- compile-time assertions prevent future Arduino-status/provisioning-status confusion.

The code runs only when the SlimeVR serial compatibility commands query status or set Wi-Fi credentials.

## Regression coverage

`tools/test_slimevr_wifi_provisioning_0023gh_policy.py` verifies:

- all six protocol values;
- the saved-attempt/server-attempt distinction and the failure/success mappings;
- exact `SET WIFI` and `SET BWIFI` success strings;
- removal of the incompatible `saved to NVS` text;
- persistent save followed by immediate non-blocking completion;
- aggregate `check_all.py` registration;
- host compilation of the compatibility translation unit.

PlatformIO Production/Production-Diagnostic builds remain the authoritative ESP32 composition/link gates. Slim excludes the serial compatibility translation unit by profile contract.

## Measured host direction

Compared with the exact 0023gg predecessor using host GCC `-Os`:

- compatibility object text: `+6 bytes`;
- data: `0 bytes`;
- BSS: `+1 byte` for the server-credential-attempt latch;
- `wifiStateCode()` stack: `16 bytes -> 16 bytes`;
- `setWifiCredentials()` stack: `96 bytes -> 96 bytes`.

## Validation completed

Passed:

- `test_slimevr_wifi_provisioning_0023gh_policy.py`;
- native and Production-profile host compilation of the compatibility source;
- `test_slimevr_session_contract_policy.py`;
- predecessor `test_calibration_0023gg_policy.py` and its targeted native regressions;
- source-filter, profile-matrix and documentation validators;
- check-all policy and aggregation-policy tests;
- calibration integration policy;
- `git diff --check`.

PlatformIO is not installed in the audit environment, so the user's mandatory `check_all.py --clean --require-pio` remains the final ESP32 build/link gate.
