#include "test_common.hpp"

#include <cstddef>
#include <cstdint>

#include "Preferences.h"
#include "config/tracker_config_store.hpp"
#include "sensor/gyro_temperature_compensation.hpp"
#include "sensor/mag_axis_alignment.hpp"

using namespace tracker;

namespace {

#include "cases/config_storage/fixtures.hpp"
#include "cases/config_storage/durability.hpp"
#include "cases/config_storage/candidates.hpp"
#include "cases/config_storage/migration.hpp"
#include "cases/config_storage/admission.hpp"

} // namespace

int main() {
    TestContext ctx;
    testDualSlotSaveAndLoad(ctx);
    testTornInactiveWriteKeepsOldSelector(ctx);
    testTornSelectorKeepsPreviousActive(ctx);
    testCorruptSelectedSlotFallsBack(ctx);
    testSelectorLossUsesNewestExplicitlyCommittedGeneration(ctx);
    testLegacyV1SelectorLossUsesConservativeOlderGeneration(ctx);
    testLegacyMigration(ctx);
    testCandidateDoesNotBecomeActiveBeforeCommit(ctx);
    testPromotedCandidateMetadataWriteCanRetryWithoutRepromotion(ctx);
    testPreparedPromotionAbortedByRebootKeepsActive(ctx);
    testCandidateSignatureMismatchBlocksPromotion(ctx);
    testCandidateQualityAndWearGates(ctx);
    testCandidateSurvivesUnrelatedActiveConfigChange(ctx);
    testCandidateBecomesStaleAfterActiveCalibrationChange(ctx);
    testPreparedNewerSlotNeverBecomesFallback(ctx);
    testAbortInvalidatesPreparedSlot(ctx);
    testPromotionRejectsDivergedRuntimeCalibration(ctx);
    testPromotionPreservesUnrelatedActiveSettingsAndMeasuredQuality(ctx);
    testMigrationCleanupFailureKeepsMigratedConfig(ctx);
    testManualMigrationNeverOverwritesActive(ctx);
    testSelectorReadbackFailureReconcilesCommittedWrite(ctx);
    testPersistentSelectorReadFailureReportsUncertain(ctx);
    testLoadStatusDistinguishesStorageFailure(ctx);
    testEmptyNvsLifecycle(ctx);
    testCorruptActiveStorageIsNotClassifiedAsEmpty(ctx);
    testTransientBootReadFailureBlocksWritesUntilAuthoritativeLoad(ctx);
    testAuthoritativeLoadRequiresApplyConfirmationBeforeWrites(ctx);
    testFirstSaveReadbackFailureNeverBecomesAuthoritative(ctx);
    testInterruptedLegacyMigrationRecoversFromStillValidLegacyBlob(ctx);
    testSelectorIsAuthoritativeWhenCommitMarkerWriteFails(ctx);
    testNoOpSavePreservesGenerationAndCandidate(ctx);
    testRuntimeSnapshotNoOpAndProvenancePreservation(ctx);
    testV3CandidateFreshAcrossEvidenceAndTempPolicyChanges(ctx);
    testV2CandidateRevisionCompatibility(ctx);
    testLoadUpgradesLegacyV1ActiveSlotToV3(ctx);
    testLegacyV1CandidateUsesGenerationFreshnessContract(ctx);
    testCorruptCandidateDoesNotBreakActiveConfig(ctx);
    testDegradedStateBlocksCandidateDiscardButAllowsFullErase(ctx);
    testDormantFrameBytesDoNotChangeSensorSignature(ctx);
    testPromotionRejectsDivergedRuntimeSensorContract(ctx);
    testMeasuredAxisCandidateBeatsUnmeasuredValidAlignment(ctx);
    testSolverMeasuredAxisCandidateUsesNormalStorePromotionPath(ctx);
    testGenerationWrapComparison(ctx);
    testDeployedV3MigrationKeepsCalibrations(ctx);
    testSafeModeWriteInhibit(ctx);
    testSafeModeLoadPerformsNoRepairOrMigrationWrites(ctx);
    testTransitionalV2MigratesWithoutLosingMagOrCalibration(ctx);
    testTransitionalV2ReadOnlyMigrationIsVolatileAndPreservesSource(ctx);
    testInterruptedTransitionalV2MigrationKeepsOldGood(ctx);
    testTransitionalV2ImpossibleCalibrationStillFailsClosed(ctx);
    testRollbackFailureDoesNotApplyOutput(ctx);
    testCurrentSlotRequiresSemanticAdmission(ctx);
    testPreparedAuthoritativeCommitKeepsOldGoodUntilSelectorCommit(ctx);
    return ctx.finish("test_config_storage");
}
