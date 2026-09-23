#include "test_common.hpp"
#include "dev05/reference_rotation.hpp"
#include "dev05/observation_contracts.hpp"
#include "sensor/ahrs_6dof.hpp"
#include "build_config/tracking_tuning.hpp"
#include "sensor/mag_heading.hpp"
#include "sensor/mag_yaw_correction.hpp"
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <vector>
using namespace dev05;
#if defined(_WIN32)
constexpr const char* hostPlatform="windows";
#elif defined(__linux__)
constexpr const char* hostPlatform="linux";
#else
constexpr const char* hostPlatform="other";
#endif
static Q oq(const tracker::Quat& q) { return {q.w,q.x,q.y,q.z}; }
static tracker::Quat fq(Q q) { return {static_cast<float>(q[0]),static_cast<float>(q[1]),static_cast<float>(q[2]),static_cast<float>(q[3])}; }
static tracker::Vec3 fv(V v) { return {static_cast<float>(v[0]),static_cast<float>(v[1]),static_cast<float>(v[2])}; }
struct InputHash {
    uint64_t value=14695981039346656037ULL;
    void integer(uint64_t v) { for(int j=0;j<8;++j) { value^=(v>>(8*j))&255; value*=1099511628211ULL; } }
    void number(float v) { uint32_t bits=0; std::memcpy(&bits,&v,sizeof(bits)); integer(bits); }
    void vector(tracker::Vec3 v) { number(v.x); number(v.y); number(v.z); }
    std::string hex() const { std::ostringstream s; s<<std::hex<<std::setw(16)<<std::setfill('0')<<value; return s.str(); }
};
class ConfigJson {
    std::ostringstream stream;
    bool first = true;
public:
    ConfigJson() { stream.imbue(std::locale::classic()); stream << std::setprecision(9) << "{"; }
    void field(const char* name, double value) {
        if (!std::isfinite(value)) throw std::runtime_error("invalid configuration");
        if (!first) stream << ",";
        first = false;
        stream << "\"" << name << "\":" << value;
    }
    std::string finish() { return stream.str() + "}"; }
};
static std::string config(const tracker::Ahrs6DofConfig& c) {
    ConfigJson out;
    out.field("worldUp.x", c.worldUp.x);
    out.field("worldUp.y", c.worldUp.y);
    out.field("worldUp.z", c.worldUp.z);
    out.field("minDtS", c.minDtS);
    out.field("maxDtS", c.maxDtS);
    out.field("gyroDeadbandRadS", c.gyroDeadbandRadS);
    out.field("accelKp", c.accelKp);
    out.field("maxAccelCorrectionRadPerUpdate", c.maxAccelCorrectionRadPerUpdate);
    out.field("accelNormGoodErrorG", c.accelNormGoodErrorG);
    out.field("accelNormBadErrorG", c.accelNormBadErrorG);
    out.field("accelInnovationGoodRad", c.accelInnovationGoodRad);
    out.field("accelInnovationBadRad", c.accelInnovationBadRad);
    out.field("accelNormVarianceGoodG2", c.accelNormVarianceGoodG2);
    out.field("accelNormVarianceBadG2", c.accelNormVarianceBadG2);
    out.field("accelNormVarianceAlpha", c.accelNormVarianceAlpha);
    out.field("gyroNormAccelTrustGoodRadS", c.gyroNormAccelTrustGoodRadS);
    out.field("gyroNormAccelTrustBadRadS", c.gyroNormAccelTrustBadRadS);
    out.field("clampLargeDt", c.clampLargeDt);
    out.field("accelCorrectionEnabled", c.accelCorrectionEnabled);
    out.field("adaptiveAccelCorrection", c.adaptiveAccelCorrection);
    out.field("normalizeEvery", c.normalizeEvery);
    out.field("accelCorrectionDivisor", tracker::cfg::AHRS_ACCEL_CORRECTION_DIVISOR);
    return out.finish();
}
struct Metric {
    std::vector<double> orientation,tilts,yaws;
    double norm=0,step=0; Q previous={1,0,0,0}; bool first=true;
    void sample(Q q,Q truth) {
        double n=0; for(double v:q) n+=v*v;
        if(!std::isfinite(n)||n<1e-12) throw std::runtime_error("invalid publication");
        norm=std::max(norm,std::abs(std::sqrt(n)-1));
        orientation.push_back(error(q,truth)); tilts.push_back(tilt(q,truth)); yaws.push_back(yawError(q,truth));
        if(!first) step=std::max(step,error(q,previous));
        previous=q; first=false;
    }
    static double rms(const std::vector<double>& v) { double sum=0; for(double x:v)sum+=x*x; return std::sqrt(sum/static_cast<double>(v.size())); }
    static double p95(std::vector<double> v) { std::sort(v.begin(),v.end()); return v[(95*v.size()+99)/100-1]; }
    void emit(const char* id,const char* role,const InputHash& hash,const std::string& cfg,
              bool requirementMet,double recoveryMs=-1,double lagMs=0,uint32_t rejects=0,double evidenceLoss=0) const {
        std::ostringstream s; s.imbue(std::locale::classic()); s<<std::setprecision(12);
        s<<"DEV05_RESULT {\"schema\":1,\"suite\":\"dev05-v1\",\"scenario\":\""<<id
         <<"\",\"role\":\""<<role<<"\",\"compiler\":\""<<__VERSION__<<"\",\"pointer_bits\":"<<sizeof(void*)*8<<",\"platform\":\""<<hostPlatform<<"\""
         <<",\"input_fnv1a64\":\""<<hash.hex()<<"\",\"config\":"<<cfg<<",\"samples\":"<<orientation.size()
         <<",\"requirement_met\":"<<(requirementMet?"true":"false")<<",\"recovery_ms\":";
        if(recoveryMs<0)s<<"null";else s<<recoveryMs;
        s<<",\"rejects\":"<<rejects<<",\"metrics\":{\"orientation_rms_deg\":"<<rms(orientation)
         <<",\"orientation_p95_deg\":"<<p95(orientation)<<",\"orientation_max_deg\":"<<*std::max_element(orientation.begin(),orientation.end())
         <<",\"tilt_rms_deg\":"<<rms(tilts)<<",\"yaw_rms_deg\":"<<rms(yaws)
         <<",\"final_orientation_deg\":"<<orientation.back()<<",\"norm_max_error\":"<<norm
         <<",\"evidence_loss\":"<<evidenceLoss<<",\"lag_abs_ms\":"<<std::abs(lagMs)<<"},\"max_output_step_deg\":"<<step<<"}";
        std::puts(s.str().c_str());
    }
};
static double noise(uint32_t& seed) { seed^=seed<<13; seed^=seed>>17; seed^=seed<<5; return static_cast<double>(seed)/4294967295.0*2-1; }
enum class AhrsCase { Static, GyroAxis, GyroNoncommuting, CleanMultiAxis, YawSine, MissingAccel, DynamicAccel, Tilt25, Tilt90, NoiseBias, ContinuousDynamic, SaturatedAccel, TimestampJitter, Gyro60s };
enum class MagCase { CleanTilted, Disturbance, Stale, UntrustedTilt, Yaw180, ContinuousInterference, Yaw90 };
// Sample timestamps use an integer fractional clock: exactly 960 samples/second.
static void ahrsScenario(TestContext& ctx,AhrsCase mode,const char* id) {
    tracker::Ahrs6DofConfig cfg; cfg.clampLargeDt=false;
    cfg.accelCorrectionEnabled=(mode != AhrsCase::GyroAxis && mode != AhrsCase::GyroNoncommuting && mode != AhrsCase::YawSine && mode != AhrsCase::Gyro60s);
    tracker::Ahrs6Dof ahrs(cfg); Q truth={1,0,0,0};
    const Q initial=(mode == AhrsCase::Tilt90?exp({pi/2,0,0}):mode == AhrsCase::Tilt25?exp({25/degrees,0,0}):truth);
    CHECK(ctx,ahrs.reset(fq(initial),1000000));
    Metric metrics; InputHash inputs; uint32_t seed=0x5eed1234; uint64_t previous=1000000;
    FirstSampleDwell settling(240, 1000000);
    double sinA=0,cosA=0,sinT=0,cosT=0;
    const uint32_t count=mode == AhrsCase::Tilt90?960*30:mode == AhrsCase::Gyro60s?960*60:960*12;
    for(uint32_t i=1;i<=count;++i) {
        const uint64_t now=1000000+static_cast<uint64_t>(i)*1000000/960 + ((mode == AhrsCase::TimestampJitter && (i%2))?50:0);
        const double t=static_cast<double>(now-1000000)/1e6;
        const double dt=static_cast<double>(now-previous)/1e6;
        V omega{};
        if(mode == AhrsCase::GyroAxis || mode == AhrsCase::TimestampJitter)omega={0.37,-0.51,0.776};
        if(mode == AhrsCase::Gyro60s)omega={3.7,-5.1,7.76};
        if(mode == AhrsCase::GyroNoncommuting || mode == AhrsCase::CleanMultiAxis || mode == AhrsCase::MissingAccel || mode == AhrsCase::DynamicAccel || mode == AhrsCase::ContinuousDynamic || mode == AhrsCase::SaturatedAccel) {
            const int axis=static_cast<int>((i-1)/1920)%3;
            omega[axis]=0.4;
        }
        if(mode == AhrsCase::YawSine) {
            const double oldT=static_cast<double>(previous-1000000)/1e6;
            omega[2]=0.5*(std::sin(2*pi*t)-std::sin(2*pi*oldT))/dt;
        }
        truth=unit(product(truth,exp(scale(omega,dt))));
        V accel=matrix(truth).inverse({0,0,1});
        if(mode == AhrsCase::NoiseBias) { for(int j=0;j<3;++j) { omega[j]+=0.001*noise(seed); accel[j]+=0.003*noise(seed); } omega[2]+=0.002; }
        if(mode == AhrsCase::MissingAccel && t>=2 && t<5)accel={0,0,0};
        if((mode == AhrsCase::DynamicAccel && t>=2 && t<5)||mode == AhrsCase::ContinuousDynamic)accel={2,0,1};
        if(mode == AhrsCase::SaturatedAccel && t>=2 && t<5)accel={20,0,1}; // provably outside gravity gate
        const auto gyro=fv(omega),a=fv(accel);
        inputs.integer(now); inputs.vector(gyro); inputs.vector(a);
        const int failuresBefore=ctx.failures;
        const bool updated=ahrs.update(gyro,a,now);
        CHECK(ctx,updated); if(!updated)break;
        const Q q=oq(ahrs.quaternion());
        if ((mode == AhrsCase::Tilt25 || mode == AhrsCase::Tilt90) && !metrics.first) {
            CHECK(ctx,error(q,metrics.previous)<=static_cast<double>(ahrs.config().maxAccelCorrectionRadPerUpdate)*degrees+0.0001);
        }
        metrics.sample(q,truth);
        CHECK(ctx,metrics.norm<1e-4);
        if(((mode == AhrsCase::DynamicAccel||mode == AhrsCase::SaturatedAccel) && t>=2.1 && t<5) || (mode == AhrsCase::ContinuousDynamic && t>0.1)) CHECK(ctx,!ahrs.stats().lastAccelGate.accepted);
        if(mode == AhrsCase::Tilt25||mode == AhrsCase::Tilt90) settling.observe(now, tilt(q,truth)<0.5);
        if(mode == AhrsCase::YawSine) { sinA+=yaw(q)*std::sin(2*pi*t); cosA+=yaw(q)*std::cos(2*pi*t); sinT+=yaw(truth)*std::sin(2*pi*t); cosT+=yaw(truth)*std::cos(2*pi*t); }
        if(ctx.failures!=failuresBefore) {
            std::fprintf(stderr,"FAIL scenario=%s sample=%u t_us=%llu\n",id,i,static_cast<unsigned long long>(now));
            break;
        }
        previous=now;
    }
    if(metrics.orientation.empty()) throw std::runtime_error("scenario produced no samples");
    const auto firstWindow = settling.firstWindowStartUs();
    const double recovery = firstWindow ? static_cast<double>(*firstWindow - 1000000) / 1000 : -1;
    const bool future=mode == AhrsCase::Tilt90;
    const bool met= future||mode == AhrsCase::Tilt25 ? (recovery>=0 && metrics.tilts.back()<0.5) : metrics.orientation.back()< (mode == AhrsCase::NoiseBias?2.0:1.0);
    if(!future && !met) std::fprintf(stderr,"FAIL scenario=%s sample=%u final_error_deg=%.9g recovery_ms=%.9g\n",id,count,metrics.orientation.back(),recovery);
    if(!future) CHECK(ctx,met);
    const double lag=mode == AhrsCase::YawSine?std::remainder(std::atan2(cosA,sinA)-std::atan2(cosT,sinT),2*pi)/(2*pi)*1000:0;
    metrics.emit(id,future?"roadmap-0029":"acceptance",inputs,config(ahrs.config()),met,recovery,lag,ahrs.stats().accelRejectedCount);
}
static void timeAndInvalid(TestContext& ctx) {
    tracker::Ahrs6DofConfig cfg; cfg.clampLargeDt=false; cfg.accelCorrectionEnabled=false;
    tracker::Ahrs6Dof a(cfg); CHECK(ctx,a.reset({},4294967000ULL));
    const tracker::Vec3 gyro(0,0,0.2f),up(0,0,1); CHECK(ctx,a.update(gyro,up,4294968000ULL));
    const Q old=oq(a.quaternion()); const auto integrated=a.stats().lastIntegratedTimestampUs;
    CHECK(ctx,!a.update(gyro,up,4294968000ULL)); CHECK(ctx,!a.update(gyro,up,4294967000ULL));
    CHECK(ctx,a.stats().lastIntegratedTimestampUs==integrated); CHECK(ctx,error(old,oq(a.quaternion()))<1e-12);
    CHECK(ctx,!a.update(gyro,up,4295968000ULL)); CHECK(ctx,a.stats().lastIntegratedTimestampUs==integrated);
    CHECK(ctx,a.update(gyro,up,4295969000ULL));
    a.rebaseTimestamp(4296969000ULL); CHECK(ctx,a.update(gyro,up,4296970000ULL));
    const float nan=std::numeric_limits<float>::quiet_NaN(); const Q before=oq(a.quaternion());
    CHECK(ctx,!a.update({nan,0,0},up,4296971000ULL)); CHECK(ctx,error(before,oq(a.quaternion()))<1e-12);
    constexpr uint64_t sampleUs = 4296971000ULL;
    const auto previousIntegratedUs = a.stats().lastIntegratedTimestampUs;
    const Q expected = product(before,exp({0,0,static_cast<double>(gyro.z)*0.001}));
    const bool updated = a.update(gyro,{nan,0,1},sampleUs);
    const Q actual = oq(a.quaternion());
    const bool propagates = gyroPropagationObserved(updated, actual, expected,
        previousIntegratedUs, a.stats().lastIntegratedTimestampUs, sampleUs);
    Metric m; m.sample(actual,expected); InputHash h; h.integer(sampleUs); h.vector(gyro); h.vector({nan,0,1});
    m.emit("invalid_accel_gyro_continuity","roadmap-0029",h,config(a.config()),propagates);
}
static void magScenario(TestContext& ctx,MagCase mode,const char* id) {
    tracker::MagHeadingEstimator heading; tracker::MagYawCorrectionController controller;
    tracker::MagYawCorrectionConfig cfg; cfg.applyEnabled=true;
    const Q truth=exp({0.3,-0.2,0}); Q q=product(exp({0,0,mode == MagCase::Yaw180?pi:mode == MagCase::Yaw90?pi/2:0.2}),truth);
    Metric m; InputHash hash; double recovered=-1; uint32_t applied=0;
    for(uint32_t i=1;i<=60*30;++i) {
        // Exercise uint32 wrap with a clean sensor timestamp in a separate domain.
        const uint32_t now=0xffffe000u+i*1000/60;
        const bool disturbed=i>=120 && (i<360 || mode == MagCase::ContinuousInterference);
        tracker::MagYawCorrectionInput in; in.nowMs=now; in.referenceValid=true; in.referenceWorldYawRad=static_cast<float>(pi); in.mag.valid=true;
        in.mag.trusted=!((mode == MagCase::Disturbance || mode == MagCase::ContinuousInterference) && disturbed); in.magTrustedForUse=in.mag.trusted;
        in.mag.receivedMs=now-((mode == MagCase::Stale && disturbed)?1000u:0u); in.mag.t_us=1000000ULL+i*1000000ULL/60; in.mag.seq=i;
        in.mag.body=fv(matrix(truth).inverse({300,0,150}));
        heading.update(in.mag,fq(q),{},now,in.heading);
        in.accelTrust=(mode == MagCase::UntrustedTilt && disturbed)?0.0f:1.0f;
        in.fieldReliable=in.mag.trusted; in.fieldStableMs=10000;
        hash.integer(now);hash.integer(in.mag.receivedMs);hash.vector(in.mag.body);hash.integer(in.mag.trusted);hash.number(in.accelTrust);
        const int failuresBefore=ctx.failures;
        tracker::MagYawCorrectionOutput out; controller.update(in,cfg,out);
        if(disturbed && ((mode == MagCase::Disturbance || mode == MagCase::Stale || mode == MagCase::UntrustedTilt)||mode == MagCase::ContinuousInterference)) CHECK(ctx,!out.applyAllowed);
        if(out.applyAllowed) {
            const double oldError=error(q,truth);
            // Same world-yaw multiplication convention as app; wrapper, not full app replay.
            q=oq(tracker::Quat::fromAxisAngle(tracker::Vec3::unitZ(),out.correctionStepRad)*fq(q));
            CHECK(ctx,error(q,truth)<=oldError+0.00005); CHECK(ctx,std::abs(out.correctionStepDeg)<=0.25001f);
            CHECK(ctx,tilt(q,truth)<0.001); ++applied;
            if(i>=360 && recovered<0) recovered=static_cast<double>(i-360)*1000/60;
        }
        m.sample(q,truth);
        if(ctx.failures!=failuresBefore) {
            std::fprintf(stderr,"FAIL scenario=%s sample=%u now_ms=%u\n",id,i,now);
            break;
        }
    }
    const bool met=applied>0 && (mode == MagCase::ContinuousInterference || recovered>=0);
    if(mode != MagCase::Yaw180)CHECK(ctx,met);
    ConfigJson c;
    c.field("enabled", cfg.enabled);
    c.field("applyEnabled", cfg.applyEnabled);
    c.field("maxInnovationDeg", cfg.maxInnovationDeg);
    c.field("maxMagAgeMs", cfg.maxMagAgeMs);
    c.field("horizontalNormGood", cfg.horizontalNormGood);
    c.field("horizontalNormBad", cfg.horizontalNormBad);
    c.field("gyroNormGoodDps", cfg.gyroNormGoodDps);
    c.field("gyroNormBadDps", cfg.gyroNormBadDps);
    c.field("accelTrustGood", cfg.accelTrustGood);
    c.field("accelTrustBad", cfg.accelTrustBad);
    c.field("requireAccelTrusted", cfg.requireAccelTrusted);
    c.field("timeConstantS", cfg.timeConstantS);
    c.field("maxCorrectionRateDegS", cfg.maxCorrectionRateDegS);
    c.field("maxCorrectionStepDeg", cfg.maxCorrectionStepDeg);
    c.field("fallbackDtS", cfg.fallbackDtS);
    c.field("gyroMovingCooldownMs", cfg.gyroMovingCooldownMs);
    c.field("accelBadCooldownMs", cfg.accelBadCooldownMs);
    c.field("magDisturbanceCooldownMs", cfg.magDisturbanceCooldownMs);
    c.field("reacquisitionEnabled", cfg.reacquisitionEnabled);
    c.field("reacquireInnovationMaxDeg", cfg.reacquireInnovationMaxDeg);
    c.field("reacquireMinFieldStableMs", cfg.reacquireMinFieldStableMs);
    c.field("reacquireMaxHeadingRateDegS", cfg.reacquireMaxHeadingRateDegS);
    c.field("reacquireTimeConstantS", cfg.reacquireTimeConstantS);
    c.field("reacquireMaxCorrectionRateDegS", cfg.reacquireMaxCorrectionRateDegS);
    c.field("reacquireMaxCorrectionStepDeg", cfg.reacquireMaxCorrectionStepDeg);
    const tracker::MagHeadingConfig headingConfig;
    c.field("heading.requireTrustedMag", headingConfig.requireTrustedMag);
    c.field("heading.minHorizontalNorm", headingConfig.minHorizontalNorm);
    m.emit(id,mode == MagCase::Yaw180?"roadmap-0030":"acceptance",hash,c.finish(),met,recovered,0,controller.stats().gateClosedCount);
}
static void accelEvidenceDiagnostics(TestContext& ctx) {
    tracker::Ahrs6DofConfig cfg;
    tracker::Ahrs6Dof missing(cfg);
    CHECK(ctx, missing.reset({}, 1000000));
    InputHash missingHash;
    Metric missingMetrics;
    double meanBefore = 0;
    for (uint32_t i = 1; i <= 1920; ++i) {
        const uint64_t now = 1000000 + static_cast<uint64_t>(i) * 1000000 / 960;
        const tracker::Vec3 accel = i <= 960 ? tracker::Vec3::unitZ() : tracker::Vec3::zero();
        missingHash.integer(now); missingHash.vector(accel);
        CHECK(ctx, missing.update({}, accel, now));
        missingMetrics.sample(oq(missing.quaternion()), {1,0,0,0});
        if (i == 960) meanBefore = missing.stats().accelNormMeanG;
    }
    const double meanDelta = std::abs(static_cast<double>(missing.stats().accelNormMeanG) - meanBefore);
    missingMetrics.emit("missing_accel_statistics", "roadmap-0029", missingHash,
                        config(missing.config()), meanDelta < 1e-6, -1, 0, 0, meanDelta);

    // Sweep every phase of a 60 Hz yaw callback against the accel aggregation.
    // The wrapper uses the existing setQuaternion entry, not a second estimator.
    Metric yawMetrics; InputHash yawHash; uint32_t lost = 0;
    constexpr uint32_t gyroPerMag = 16;
    for (uint32_t phase = 0; phase < gyroPerMag; ++phase) {
        tracker::Ahrs6Dof baseline(cfg), withYaw(cfg);
        CHECK(ctx, baseline.reset({},1000000)); CHECK(ctx,withYaw.reset({},1000000));
        Q expected={1,0,0,0};
        for(uint32_t i=1; i<=960; ++i) {
            const uint64_t now=1000000+static_cast<uint64_t>(i)*1000000/960;
            CHECK(ctx,baseline.update({},tracker::Vec3::unitZ(),now));
            CHECK(ctx,withYaw.update({},tracker::Vec3::unitZ(),now));
            yawHash.integer(now); yawHash.integer(phase);
            if(i%gyroPerMag==phase) {
                constexpr float yawStep=1e-5f;
                const auto correction=tracker::Quat::fromAxisAngle(tracker::Vec3::unitZ(),yawStep);
                CHECK(ctx,withYaw.setQuaternion(correction*withYaw.quaternion()));
                expected=product(exp({0,0,static_cast<double>(yawStep)}),expected);
            }
            yawMetrics.sample(oq(withYaw.quaternion()),expected);
        }
        const auto baseUpdates=baseline.stats().accelUpdateCount;
        const auto yawUpdates=withYaw.stats().accelUpdateCount;
        if(yawUpdates<baseUpdates)lost+=baseUpdates-yawUpdates;
    }
    yawMetrics.emit("yaw_accel_evidence_phases","roadmap-0029",yawHash,config(cfg),lost==0,
                    -1,0,0,static_cast<double>(lost));
}
int main() {
    TestContext ctx;
    try {
        const char* names[]={"static","gyro_axis","gyro_noncommuting","clean_multi_axis","yaw_sine","missing_accel","dynamic_accel","tilt_25_relock","tilt_90_relock","seeded_noise_bias","continuous_dynamic_accel","saturated_accel","timestamp_jitter","gyro_60s"};
        for(int i=0;i<14;++i)ahrsScenario(ctx,static_cast<AhrsCase>(i),names[i]);
        timeAndInvalid(ctx);
        accelEvidenceDiagnostics(ctx);
        const char* mags[]={"mag_clean_tilted","mag_disturbance_return","mag_stale_return","mag_untrusted_tilt","mag_180_relock","mag_continuous_interference","mag_90_relock"};
        for(int i=0;i<7;++i)magScenario(ctx,static_cast<MagCase>(i),mags[i]);
    } catch(const std::exception& e) { std::fprintf(stderr,"DEV05 fatal: %s\n",e.what()); CHECK(ctx,false); }
    return ctx.finish("test_dev05_algorithm_scenarios");
}
