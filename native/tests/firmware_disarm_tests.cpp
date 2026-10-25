#include <memory>
#include "gtest/gtest.h"
#include "simple_flight/AirSimSimpleFlightBoard.hpp"
#include "simple_flight/AirSimSimpleFlightCommLink.hpp"
#include "simple_flight/AirSimSimpleFlightEstimator.hpp"
#include "simple_flight/AirSimSimpleFlightEstimatorFW.hpp"
#include "simple_flight/firmware/Firmware.hpp"

namespace sim = microsoft::projectairsim;
class TestBoard : public sim::AirSimSimpleFlightBoard {
 public:
  using AirSimSimpleFlightBoard::AirSimSimpleFlightBoard;
  int64_t ms = 0;
  int64_t Millis() const override { return ms; }
  int64_t Micros() const override { return ms * 1000; }
};

TEST(SimpleFlightDisarm, LateCommandAndTimeoutCannotRestartMotors) {
  simple_flight::Params params;
  TestBoard board(&params);
  sim::AirSimSimpleFlightCommLink link;
  sim::AirSimSimpleFlightEstimator estimator;
  sim::AirSimSimpleFlightEstimatorFW fixedwing;
  sim::Kinematics kinematics;
  kinematics.pose.orientation = sim::Quaternion::Identity();
  kinematics.pose.position = sim::Vector3(0,0,-5);
  kinematics.twist.linear = sim::Vector3(0,.5,0);
  kinematics.twist.angular.setZero();
  kinematics.accels.linear.setZero();
  kinematics.accels.angular.setZero();
  board.SetGroundTruthKinematics(&kinematics);
  estimator.SetGroundTruthKinematics(&kinematics);
  simple_flight::Firmware firmware(&params,&board,&link,&estimator,&fixedwing);
  firmware.Reset();
  std::string message;
  auto& api = firmware.OffboardApi();
  ASSERT_TRUE(api.RequestApiControl(message));
  ASSERT_TRUE(api.Arm(message));
  auto goal = simple_flight::Axis4r(0,0,0,.6);
  auto mode = simple_flight::GoalMode::GetAllRateMode();
  ASSERT_TRUE(api.SetGoalAndMode(&goal,&mode,message));
  for (int i=0;i<20;++i) { board.ms+=5;firmware.Update(); }
  EXPECT_GT(board.GetAvgMotorOutput(),.1f);
  ASSERT_TRUE(api.Disarm(message));
  // An already-running command can finish after Disarm. Even an explicit
  // high-throttle stale goal must be unable to bypass the output interlock.
  ASSERT_TRUE(api.SetGoalAndMode(&goal,&mode,message));
  for (int i=0;i<1000;++i) {
    board.ms+=5; firmware.Update();
    for (int motor=0;motor<params.motor.motor_count;++motor)
      ASSERT_FLOAT_EQ(board.GetMotorControlSignal(motor),0.0f);
    ASSERT_NE(api.GetVehicleState(),simple_flight::VehicleStateType::kArmed);
    ASSERT_NE(api.GetVehicleState(),simple_flight::VehicleStateType::kActive);
  }
  ASSERT_TRUE(api.Arm(message));
  ASSERT_TRUE(api.SetGoalAndMode(&goal,&mode,message));
  board.ms+=5;firmware.Update();
  EXPECT_GT(board.GetAvgMotorOutput(),.1f);
}
