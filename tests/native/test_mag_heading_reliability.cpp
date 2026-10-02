#include "test_common.hpp"

#include <initializer_list>
#include <limits>

#include "sensor/frame_transform.hpp"
#include "sensor/mag_axis_alignment.hpp"
#include "sensor/mag_field_reliability.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"

using namespace tracker;

#include "cases/magnetic/heading_admission.hpp"
#include "cases/magnetic/disturbance_recovery.hpp"
#include "cases/magnetic/alignment_collection.hpp"
#include "cases/magnetic/alignment_solver.hpp"

int main() {
    TestContext ctx;
    testHeadingRejectsInvalidQuaternionWithoutIdentityFallback(ctx);
    testFieldReliabilityFailsClosedAcrossChangedEnvironment(ctx);
    testHighDipHealthyFieldUsesAdaptiveHorizontalTrust(ctx);
    testHighDipLegacyHeadingStepGateScalesWithObservability(ctx);
    testUnobservableHorizontalHeadingSkipsDirectionalStepGate(ctx);
    testNearVerticalFieldRemainsFailClosedForYaw(ctx);
    testThirtyMinuteHighDipNoiseAndDisturbanceRecovery(ctx);
    testYawGateUsesSameAdaptiveHorizontalTrust(ctx);
    testModerateStationaryHeadingJumpStaysFailClosed(ctx);
    testAhrsYawFrameJumpDoesNotMasqueradeAsFieldJump(ctx);
    testStationaryFieldReturnRecoversAfterAhrsYawDrift(ctx);
    testStationaryFieldShortcutDisablesAfterPhysicalMotion(ctx);
    testStationaryJumpAtWindowBoundaryStillLatches(ctx);
    testSlowStationaryYawDriftRemainsCorrectable(ctx);
    testFastButPlausibleYawDriftDoesNotSelfLatch(ctx);
    testMaximumNormalYawCorrectionDoesNotSelfLatch(ctx);
    testRealRotationDoesNotTriggerStationaryJumpLatch(ctx);
    testVerySlowPhysicalRotationDoesNotTriggerJumpLatch(ctx);
    testFilteredHeadingRateAllowsNoisySixtyHertzReacquisition(ctx);
    testIntervalBuilderUsesTimestampCoherentGyroEndpoints(ctx);
    testAllProperSignedPermutationMountingsConverge(ctx);
    testReflectionAmbiguityRequiresRightHandedDriverContract(ctx);
    testOnlyProperSignedPermutationsAreAccepted(ctx);
    testAxisCollectorRejectsStaleGyroPair(ctx);
    testSixtyHertzCollectionSpansIndependentWindows(ctx);
    testRuntimeCollectorUsesSensorTimeAcrossProcessingBacklog(ctx);
    testDynamicAxisSolverRefinesMechanicalMisalignment(ctx);
    testSolverConfidenceIsRateNormalized(ctx);
    testSameCoarseWinnerFallsBackWhenRefinementsDisagree(ctx);
    testHoldoutRejectsTrainingOnlyFit(ctx);
    testGuidedAxisReservoirPreservesLateAxesAndPartitions(ctx);
    testDynamicAxisSolverWithHardSoftAndNearOriginRawData(ctx);
    testSolverCountsUniqueReservoirWindows(ctx);
    testSolverRejectsInvalidHardSoftTransform(ctx);
    testRuntimeCollectorUsesCalibratedDirectionAndReservoir(ctx);
    testCalibratedRawOriginRemainsUsable(ctx);
    testMagRuntimeRetainsValidatedDeviceFrameDecision(ctx);
    testDynamicAxisSolverKeepsPureRotationConstraint(ctx);
    return ctx.finish("test_mag_heading_reliability");
}
