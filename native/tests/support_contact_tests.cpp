#include "gtest/gtest.h"
#include "support_contact.hpp"
using namespace microsoft::projectairsim;
namespace {
CollisionInfo contact(float speed = 0) {
  CollisionInfo c;
  c.normal = Vector3(0,0,-1);
  c.support_valid = c.support_managed = true;
  c.time_stamp = 1000000000;
  c.contact_velocity = Vector3(speed,0,0);
  c.support_friction = 0.5f;
  return c;
}
const Vector3 g(0,0,9.81f);
}
TEST(Support, StaticGroundRemainsAtRest) {
  auto s=SolveSupportContact(Vector3::Zero(),Vector3::Zero(),g,contact(),.01,1.,.5);
  EXPECT_TRUE(s.supported); EXPECT_LT(s.velocity.norm(),1e-6);
  EXPECT_LT(s.position.norm(),1e-6);
}
TEST(Support, ConstantMovingDeckCarriesWithoutPoseBinding) {
  auto c=contact(.5);
  auto s=SolveSupportContact(Vector3::Zero(),Vector3(.5,0,0),g,c,.01,1.,.5);
  EXPECT_TRUE(s.supported); EXPECT_NEAR(s.position.x(),.005,1e-6);
  EXPECT_NEAR(s.velocity.x(),.5,1e-6);
}
TEST(Support, SuddenDeckAccelerationCausesLimitedSlip) {
  auto s=SolveSupportContact(Vector3::Zero(),Vector3::Zero(),g,contact(3),.01,1.,.5);
  EXPECT_TRUE(s.supported); EXPECT_TRUE(s.sliding);
  EXPECT_NEAR(s.velocity.x(),.5*9.81*.01,1e-6);
}
TEST(Support, ZeroFrictionDoesNotAttach) {
  auto s=SolveSupportContact(Vector3::Zero(),Vector3::Zero(),g,contact(3),.01,1.,0);
  EXPECT_TRUE(s.sliding); EXPECT_NEAR(s.velocity.x(),0,1e-7);
}
TEST(Support, MissingSupportFallsAndRetainsMomentum) {
  auto c=contact(.5); c.support_valid=false;
  auto s=SolveSupportContact(Vector3::Zero(),Vector3(.5,0,0),g,c,.01,1.,.5);
  EXPECT_FALSE(s.supported); EXPECT_NEAR(s.velocity.z(),.0981,1e-6);
  EXPECT_NEAR(s.velocity.x(),.5,1e-6);
}
TEST(Support, StaleContactCannotHoldVehicle) {
  EXPECT_FALSE(SolveSupportContact(Vector3::Zero(),Vector3::Zero(),g,contact(),.01,1.3,.5).supported);
}
TEST(Support, TakeoffReleasesUnilaterally) {
  auto s=SolveSupportContact(Vector3::Zero(),Vector3::Zero(),-g,contact(),.01,1.,.5);
  EXPECT_FALSE(s.supported); EXPECT_LT(s.velocity.z(),0);
}
TEST(Support, MovingAwayCannotBePulledBack) {
  auto s=SolveSupportContact(Vector3(0,0,-.1),Vector3::Zero(),g,contact(),.01,1.,.5);
  EXPECT_FALSE(s.supported);
}
TEST(Support, RotationContributesContactPointVelocity) {
  auto c=contact(); c.other_angular_velocity=Vector3(0,0,.1);
  c.contact_velocity=c.other_angular_velocity.cross(Vector3(2,0,0));
  auto s=SolveSupportContact(Vector3::Zero(),c.contact_velocity,g,c,.01,1.,.5);
  EXPECT_NEAR(s.velocity.y(),.2,1e-6);
  EXPECT_NEAR(s.position.y(),.002,1e-6);
}
TEST(Support, PenetrationCorrectionNeverCopiesHorizontalPose) {
  auto c=contact(); c.position=Vector3(20,10,0); c.support_gap=-.005;
  auto s=SolveSupportContact(Vector3::Zero(),Vector3::Zero(),g,c,.01,1.,.5);
  EXPECT_NEAR(s.position.x(),0,1e-7); EXPECT_NEAR(s.position.y(),0,1e-7);
  EXPECT_NEAR(s.position.z(),-.005,1e-6);
}
