// AirSimSGA: reduced planar vessel dynamics, SI units and simulation time.
// Near-level surge/sway/yaw model; this is not a six-DOF wave/buoyancy solver.
#pragma once
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace microsoft { namespace projectairsim {
struct VesselPlanarParams {
 double m11=775.,m22=990.,m33=450.;
 double du=70.,duu=135.,dv=100.,dvv=2000.,dr=50.,drr=50.;
 double anchor_z=8.8;
 void Validate() const {
  if (!(m11>0 && m22>0 && m33>0 && du>=0 && duu>=0 && dv>=0 && dvv>=0 && dr>=0 && drr>=0))
   throw std::invalid_argument("Invalid vessel effective inertia or dissipative damping");
 }
};
// x,y,yaw are earth-fixed; u,v,r are body-fixed ground-relative velocities.
using VesselPlanarState=std::array<double,6>;
using VesselPlanarForce=std::array<double,3>;
inline VesselPlanarState VesselPlanarDerivative(const VesselPlanarState& s,
 const VesselPlanarParams& p,const VesselPlanarForce& tau) {
 double u=s[3],v=s[4],r=s[5],c=std::cos(s[2]),sn=std::sin(s[2]);
 return {c*u-sn*v,sn*u+c*v,r,
  (tau[0]-(p.du+p.duu*std::abs(u))*u+p.m22*v*r)/p.m11,
  (tau[1]-(p.dv+p.dvv*std::abs(v))*v-p.m11*u*r)/p.m22,
  (tau[2]-(p.dr+p.drr*std::abs(r))*r+(p.m11-p.m22)*u*v)/p.m33};
}
inline VesselPlanarState VesselPlanarStep(const VesselPlanarState& s,
 const VesselPlanarParams& p,const VesselPlanarForce& tau,double dt) {
 if (!(dt>0 && dt<=.1)) throw std::invalid_argument("Invalid vessel simulation step");
 auto plus=[](const VesselPlanarState&a,const VesselPlanarState&b,double f){auto out=a;for(int i=0;i<6;++i)out[i]+=b[i]*f;return out;};
 auto k1=VesselPlanarDerivative(s,p,tau);
 auto k2=VesselPlanarDerivative(plus(s,k1,dt/2),p,tau);
 auto k3=VesselPlanarDerivative(plus(s,k2,dt/2),p,tau);
 auto k4=VesselPlanarDerivative(plus(s,k3,dt),p,tau);
 auto out=s;
 for(int i=0;i<6;++i) {out[i]+=dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6.;if(!std::isfinite(out[i]))throw std::runtime_error("Nonfinite vessel state");}
 out[2]=std::remainder(out[2],2*3.14159265358979323846);
 return out;
}
} }
