# pre-0024a tracking deadline and magnetic evidence hardening

## Scope

`pre-0024a_tracking_deadline_hardening` is the second standalone performance wave before AHRS `0024`. It is based on a 35.9-minute Production-Diag capture from one idle tracker on a clean network after `pre_0024_hotpath_headroom_foundation`.

The patch changes scheduling and background evidence placement only. **No IMU sample is dropped, skipped, merged or reordered.** IMU ODR, hardware timestamps, gyro integration, accel correction cadence, magnetic heading/yaw correction, quaternion math, prepared-output rate, SlimeVR packet contents and the 100 Hz target remain unchanged.

## Hardware evidence

The capture had no Wi-Fi disconnect, UDP failure, TX-pressure event, FIFO overrun, FIFO full event or tracking recovery. Rotation delivery remained 99.95%, and software sensor age was usually healthy (`processed p99=7.5 ms`, `rotation p99=10 ms`). Nevertheless:

- `network_nested` ran 335,649 times and consumed 8.11% of observed loop wall time;
- 171,458 of 218,986 due rotations were late, with 14 ms maximum lateness;
- 10 ms frame headroom p05 was zero and 31,590 frames exceeded budget;
- FIFO callback work reached 14.365 ms;
- the raw queue reached 82 samples, 80.104 ms timestamp span and 85.218 ms queue wait;
- magnetic chronological work was deferred 86,427 times, all by callback budget;
- `near_deadline_reduced_drains` fired 190,720 times.

This proves that the next limiting factor is scheduler admission and callback burst size, not RF loss or UDP failure.

## Defect 1: no-op nested network service

`TrackerApp::processFifoRuntime()` previously called `updateCriticalNetworkRuntime()` after every cooperative FIFO slice while backlog remained. Most calls were before the next 100 Hz rotation deadline. Even when no packet could be sent, `SlimeVROutputRuntime::updateCritical()` reduced a large activity signature and rechecked runtime state.

The patch adds a side-effect-free `criticalRotationServiceDue(nowMs)` gate. The app enters the nested path only when a healthy established session has an armed rotation deadline that is actually due. `updateCritical()` also uses the monotonic `rotationSendDue_` counter rather than the full activity signature to report work.

The nested path remains rotation-only. It cannot poll incoming packets, run discovery, send heartbeat/telemetry/SensorInfo, refresh Wi-Fi data or alter session configuration.

## Defect 2: callback slice overshoot

The FIFO runtime guaranteed twelve raw callbacks before checking the slice budget. A diagnostic or otherwise heavy sample callback could therefore exceed a 3.5 ms slice by more than 10 ms before pose delivery had another chance to run.

The minimum coherent micro-batch is now four callbacks. The absolute budget is checked **before dequeuing the next sample** after that minimum. The maximum remains 64. Hardware drain, raw callbacks and due magnetic callbacks still share one absolute budget.

New diagnostics report:

- sampled raw callback average/maximum;
- magnetic callback average/maximum;
- slice-budget overshoot count and maximum;
- whether urgency came from depth or timestamp span.

The software queue becomes urgent when its sensor-time span reaches 40 ms, even if its depth is below the legacy 192-sample threshold. This only selects the existing urgent processing budget; it does not delete history or change AHRS observations.

## Defect 3: background axis-learning work in the mag callback

The 60 Hz magnetic callback must synchronously perform calibrated processing, coherent gyro endpoint capture, heading/reliability and bounded yaw correction. It also performed axis-alignment reservoir observation and readiness bookkeeping. That background learner is not required to produce the current tracking pose, yet its cost blocked advancement of the raw IMU timeline and contributed to magnetic budget deferrals.

The callback now copies eligible axis-alignment evidence into a fixed eight-entry queue. One item is consumed by `serviceDeferred()` outside the FIFO/mag callback. FIFO order and the exact copied gyro endpoint, magnetic sample and sample timestamp are retained.

The queue is bounded and fail-closed:

- normal queued and processed items are counted;
- overflow drops only new background calibration evidence and is counted;
- the queue high-water mark is reported;
- every item carries the active config CRC;
- evidence from a superseded calibration/config epoch is discarded rather than mixed into the new dataset and is reported separately.

Eight entries cover the observed magnetic queue high-water of six with margin. Overflow cannot affect current yaw correction, current quaternion output or the active calibration model.

## Quality-preservation tests

Native regressions prove:

1. Repeated not-due critical calls perform no UDP send and do not advance the rotation-due counter.
2. A due critical call sends only rotation and leaves incoming FeatureFlags pending for the full outer service.
3. FIFO processing checks budget after four callbacks, records bounded overshoot and preserves every sample across subsequent passes.
4. Age urgency activates below the old depth threshold when the retained timestamp span exceeds 40 ms.
5. Recovery and explicit reset still discard only samples that predate a proven FIFO reset.
6. Magnetic callbacks retain nearest coherent raw endpoints and same-timestamp ordering.
7. Deferred axis evidence produces the same collector interval sequence and counters as direct ordered observation.
8. Deferred evidence obeys the same software-FIFO, hardware-FIFO and output-deadline admission gate as solver/storage work, so moving it out of the mag callback cannot recreate pressure one app-loop later.
9. The evidence queue is bounded, and a config-CRC change rejects all stale queued observations rather than contaminating a new model epoch.

The dedicated magnetic test links the real `MagRuntimeController`, config store types, processor, heading, reliability, yaw, axis collector, QMC and sensor-hub implementations. The compact FIFO boundary runs under ASan/UBSan; the full magnetic-controller linkage runs with the production host optimization flags.

## Expected hardware result

On the same one-tracker diagnostic workload:

- `network_nested calls` should fall from hundreds of thousands toward the actual number of due rotations serviced during FIFO catch-up;
- `network_nested pct_loop` should fall substantially below the previous 8.11%;
- raw callback maximum should become visible separately from the full FIFO callback bucket;
- slice overshoot should be bounded near one callback cost rather than twelve callback costs;
- magnetic callback time should fall because axis reservoir work is deferred;
- `runtime_mag_axis_evidence_dropped_delta` and `...stale_dropped_delta` should remain zero in normal operation;
- `runtime_mag_axis_evidence_service_deferrals_delta` may rise under pressure while the queue remains bounded, proving tracking deadlines preempt background learning;
- rotation lateness and queue span should improve without any reduction in motion sample rate, rotation target or orientation quality.

This wave does not yet implement stale-history fast-forward, central optional-work admission or controlled catch-up. Those remain dependent on the new hardware measurements.
