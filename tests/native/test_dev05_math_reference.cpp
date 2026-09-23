#include "test_common.hpp"
#include "core/math.hpp"
#include "dev05/reference_rotation.hpp"
#include <limits>
using namespace dev05;
static Q oracle(const tracker::Quat& q) { return {q.w,q.x,q.y,q.z}; }
int main() {
    TestContext ctx;
    CHECK(ctx, error({1,0,0,0},{-1,0,0,0})==0);
    CHECK(ctx, std::abs(error(exp({1e-8,0,0}),{1,0,0,0})-1e-8*degrees)<1e-12);
    CHECK(ctx, std::abs(error(exp({pi,0,0}),{1,0,0,0})-180)<1e-10);
    const V x={1,0,0}; const auto y=rodrigues({0,0,pi/2}).apply(x);
    CHECK(ctx, std::abs(y[0])<1e-15 && std::abs(y[1]-1)<1e-15);
    const Q qx=exp({pi/2,0,0}),qy=exp({0,pi/2,0});
    CHECK(ctx, error(product(qx,qy),product(qy,qx))>100); // order mutant detected
    CHECK(ctx, error(exp({0,0,0.1}),exp({0,0,-0.1}))>10); // sign mutant detected
    for(int i=0;i<=800;++i) {
        const double a=-pi+2*pi*i/800;
        const V v=scale({0.36,-0.48,0.8},a);
        const R rm=rodrigues(v),rq=matrix(exp(v));
        for(int r=0;r<3;++r) for(int c=0;c<3;++c) CHECK(ctx,std::abs(rm.m[r][c]-rq.m[r][c])<2e-15);
        const tracker::Vec3 rv(static_cast<float>(v[0]),static_cast<float>(v[1]),static_cast<float>(v[2]));
        const Q exact=exp({rv.x,rv.y,rv.z}); // compare identical float input, double computation
        CHECK(ctx, error(oracle(tracker::Quat::fromRotationVector(rv)),exact)<0.00005);
        CHECK(ctx, error(oracle(tracker::quaternionFromRotationVectorFast(rv)),exact)<0.001);
        const auto prod=tracker::Quat::fromRotationVector(rv);
        const auto actual=prod.rotate(tracker::Vec3(0.3f,-0.4f,0.5f));
        const V expected=matrix(exact).apply({0.3f,-0.4f,0.5f});
        CHECK(ctx,length({static_cast<double>(actual.x)-expected[0],static_cast<double>(actual.y)-expected[1],static_cast<double>(actual.z)-expected[2]})<1e-6);
        const V gravity=rm.inverse({0,0,1}); const V recovered=rm.apply(gravity);
        CHECK(ctx,length({recovered[0],recovered[1],recovered[2]-1})<2e-15);
    }
    for(float bad:{0.0f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()}) {
        tracker::Quat out; CHECK(ctx,!tracker::Quat(bad,0,0,0).tryNormalized(out));
    }
    return ctx.finish("test_dev05_math_reference");
}
