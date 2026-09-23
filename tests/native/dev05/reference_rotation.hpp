#pragma once
// Independent host-only oracle: no production math headers or helpers.
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace dev05 {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double degrees = 180.0 / pi;
using V = std::array<double, 3>;
using Q = std::array<double, 4>;
inline double dot(V a, V b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline V cross(V a,V b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline V scale(V a,double s) { return {s*a[0],s*a[1],s*a[2]}; }
inline double length(V a) { return std::sqrt(dot(a,a)); }
inline Q unit(Q q) {
    double n=0; for(double v:q) n+=v*v;
    if(!std::isfinite(n)||n<1e-24) throw std::runtime_error("invalid oracle quaternion");
    for(double& v:q) v/=std::sqrt(n);
    return q;
}
inline Q product(Q a,Q b) {
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline Q exp(V angle) {
    const double n=length(angle);
    if(n==0) return {1,0,0,0};
    const double k=std::sin(n/2)/n;
    return {std::cos(n/2),angle[0]*k,angle[1]*k,angle[2]*k};
}
// atan2 remains resolved for tiny angles where float acos(abs(dot)) rounds to zero.
inline double error(Q a,Q b) {
    a=unit(a); b=unit(b); b[1]=-b[1]; b[2]=-b[2]; b[3]=-b[3];
    const auto r=product(a,b);
    return 2*std::atan2(std::hypot(r[1],r[2],r[3]),std::abs(r[0]))*degrees;
}
struct R {
    double m[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    V apply(V v) const { V out{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) out[i]+=m[i][j]*v[j]; return out; }
    V inverse(V v) const { V out{}; for(int i=0;i<3;++i) for(int j=0;j<3;++j) out[i]+=m[j][i]*v[j]; return out; }
    R operator*(const R& b) const {
        R c; for(int i=0;i<3;++i) for(int j=0;j<3;++j) { c.m[i][j]=0; for(int k=0;k<3;++k) c.m[i][j]+=m[i][k]*b.m[k][j]; } return c;
    }
};
inline R matrix(Q q) {
    q=unit(q); const double w=q[0],x=q[1],y=q[2],z=q[3];
    return {{{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)},
             {2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)},
             {2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}}};
}
// Rodrigues independently checks the quaternion oracle and rotation order.
inline R rodrigues(V v) {
    const double n=length(v); if(n==0) return {};
    const V a=scale(v,1/n); const double c=std::cos(n),s=std::sin(n);
    const double k[3][3]={{0,-a[2],a[1]},{a[2],0,-a[0]},{-a[1],a[0],0}};
    R r; for(int i=0;i<3;++i) for(int j=0;j<3;++j) r.m[i][j]=(i==j?c:0)+(1-c)*a[i]*a[j]+s*k[i][j]; return r;
}
inline double angle(V a,V b) { return std::atan2(length(cross(a,b)),dot(a,b))*degrees; }
inline double tilt(Q a,Q b) { return angle(matrix(a).inverse({0,0,1}),matrix(b).inverse({0,0,1})); }
inline double yaw(Q a) { const R r=matrix(a); return std::atan2(r.m[1][0],r.m[0][0]); }
inline double yawError(Q a,Q b) { return std::abs(std::remainder(yaw(a)-yaw(b),2*pi))*degrees; }
} // namespace dev05
