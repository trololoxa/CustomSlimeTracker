# 0028b v2 config compatibility migration report

## Scope and ordering

0028b is additive and applies after 0028a. It repairs the deployed-slot
compatibility regression found on the first target boot without changing IMU,
FIFO, AHRS, accel/mag gates, calibration math, sensor cadence or network wire
behavior.

## Defect and target evidence

The target booted with `config_loaded_from_nvs=no`, default SPI 8 MHz/default
FIFO watermark 18, every calibration invalid and `mag_enabled=no`. QMC6309
remained healthy (`id=0x90`, DRDY set), and `mag enable` successfully started
the RAM stream. The magnetic hardware path was therefore not the failed owner:
the complete main config had been rejected before runtime apply.

Slot version 2 was already emitted by firmware before 0028. Those records were
loaded through `sanitize()` compatibility normalization. 0028 removed that
normalization from ordinary current-slot bootstrap, while 0028a made v2 pass
strict semantic validation inside authority resolution. A structurally valid,
CRC-valid deployed v2 record containing a known historical representation such
as an unmarked `packetFormat` was consequently rejected before its existing
migration rule could run. Boot then used RAM defaults, which disabled mag and
made all calibration appear absent. Defaults were not persisted, so avoiding a
manual save keeps the old v2 recoverable.

## Changed contract

- Strict newly written slot records are version 3.
- Versions 1 and 2 are read only as explicit migration input. Envelope, CRC,
  sensor signature and quality metadata must still be valid.
- Migration operates on a copy: `sanitize -> performance migration -> complete
  semantic validation`.
- A normalized copy is written/read back into the inactive v3 slot and the
  selector is committed before it becomes the runtime result. The source v2
  slot remains old-good through the transaction.
- Compatibility normalization does not weaken current v3 admission. A v3
  impossible config is rejected directly. A v2 config whose active calibration
  remains physically/semantically impossible after the bounded legacy rules is
  also rejected and is not written into the second slot.
- With storage writes inhibited, a normalized v1/v2 copy may be returned only
  under the existing explicitly named `loaded_legacy_read_only` volatile
  status; source NVS is unchanged.
- `config nvs` now reports each slot's version, migration requirement, semantic
  validity and semantic error. This makes future compatibility failures
  diagnosable without dumping secrets or raw config bytes.
- Commit-marker recovery treats both v2 and v3 as marker-proven generations;
  selector-loss ordering remains conservative for markerless v1.

## Regression coverage

The native storage test constructs a deployed v2 record with:

- mag driver enabled;
- valid gyro, accel, magnetic and frame calibration;
- mag yaw apply enabled;
- SPI 4 MHz and FIFO watermark 12;
- known historical packet/reserved/runtime fields.

It requires automatic migration to a committed v3 generation while preserving
all calibration and user settings. Separate tests require read-only volatile
migration with byte-identical source NVS, recovery after inactive-slot read-back
failure with byte-identical old-good, and rejection of a v2 record carrying an
impossible active accel calibration.

## Host verification

- Focused `test_config_storage`: PASS, including calibrated v2-to-v3,
  read-only, interrupted-readback and impossible-calibration cases.
- `test_storage_real_scenarios`: PASS.
- 0028, 0028a, 0028b and calibration-storage contract policies: PASS.
- Calibration-storage stack policy: PASS; no ceiling was raised.
- Source-filter, profile-matrix and documentation validators: PASS.
- `check_all --skip-native --skip-pio`: PASS, including existing sanitizer
  policy probes and 60/600-second replay gates.
- One full native invocation encountered a transient truncated assembler
  temporary file while compiling `tracker_config_store.cpp`. The same source
  was immediately compiled successfully and the affected storage executable
  was linked and run from the already-built project objects. A subsequent
  native invocation rebuilt all shared/Arduino compile-only sources and ran the
  directly affected config/schema/storage/calibration tests successfully; the
  runner session ended before unrelated remaining executables, so this is not
  reported as a new full-suite pass.

## Target smoke

Flash 0028 + 0028a + 0028b without saving defaults first.

1. Boot once and capture the complete startup header. Require
   `config_loaded_from_nvs=yes`, `config_load_status=migrated`, the previous
   SPI/FIFO values, `mag enabled in config`, QMC/hub initialization and no
   semantic/config-load error.
2. Run `config nvs`. Require the selected slot to be version 3,
   `semantic_valid=yes`, `migration_required=no` and a valid commit marker. The
   other slot may remain version 2 with `migration_required=yes`; this is the
   rollback source and must not be erased by the migration.
3. Run `health`, `cal status`, `mag status`, `mag heading`, and `mag yaw status`.
   Require the previously valid gyro/accel/mag/frame state and `mag_enabled=yes`.
   Magnetic trust/yaw may still need its normal clean dwell; it must not be
   disabled merely because the record was migrated.
4. Reboot a second time. Require `config_load_status=loaded` (not another
   migration), the same selected generation/settings and no generation churn.
5. After the second boot, run the ordinary 0028a five-minute still/motion smoke
   and check that FIFO/AHRS/mag counters progress without recovery loops.

Do not run `config save`, `config defaults ... save`, calibration clear or
factory reset before step 1 has proved recovery. If the first 0028b boot still
reports defaults, preserve NVS and capture the startup header plus `config nvs`.

## Not verified

- The supplied target evidence proves the rejection/fallback signature and
  healthy magnetic hardware, but this workspace cannot flash the corrected
  ESP32-C3 image.
- Physical v2-to-v3 NVS migration, reboot persistence, target flash/RAM delta
  and task stack high-water require the user's board/toolchain.
- Power-cut migration checkpoints require a spare tracker or NVS fault build.
