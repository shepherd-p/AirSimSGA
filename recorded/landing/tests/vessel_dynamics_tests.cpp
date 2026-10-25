#include "core_sim/vessel_planar_dynamics.hpp"
#include <iostream>
#include <cassert>
using namespace microsoft::projectairsim;
int main(){
 VesselPlanarParams p;p.Validate();VesselPlanarState s={0,0,0,0,0,0};
 // Newton acceleration from rest and analytical steady resistance at 0.5m/s.
 auto d=VesselPlanarDerivative(s,p,{68.75,0,0});assert(std::abs(d[3]-68.75/775)<1e-12);
 for(int i=0;i<60000;++i)s=VesselPlanarStep(s,p,{68.75,0,0},.005);
 assert(std::abs(s[3]-.5)<1e-6);assert(std::abs(s[4])<1e-12 && std::abs(s[5])<1e-12);
 // Loss of propulsion decelerates continuously; does not instantly stop.
 auto old=s[3];s=VesselPlanarStep(s,p,{0,0,0},.005);assert(s[3]>0 && s[3]<old);
 // Zero-damping inertia/Coriolis conserve body kinetic energy.
 p.du=p.duu=p.dv=p.dvv=p.dr=p.drr=0;s={0,0,0,.6,.15,.2};
 auto energy=[&](VesselPlanarState x){return .5*(p.m11*x[3]*x[3]+p.m22*x[4]*x[4]+p.m33*x[5]*x[5]);};
 double e=energy(s);for(int i=0;i<2000;++i)s=VesselPlanarStep(s,p,{0,0,0},.005);
 assert(std::abs(energy(s)-e)/e<1e-7);
 // Body forward at -90deg travels negative east; yaw torque accelerates yaw.
 s={0,0,-3.14159265358979323846/2,1,0,0};auto a=VesselPlanarDerivative(s,p,{0,0,100});
 assert(std::abs(a[0])<1e-12 && a[1]<-.99 && std::abs(a[5]-100./450)<1e-12);
 std::cout<<"PASS analytical acceleration, drag equilibrium, coastdown, energy conservation, body-world frame, yaw inertia\n";
}
