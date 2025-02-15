#ifndef BACKPACK_HPP
#define BACKPACK_HPP

//#include "95071e/motion_profiler.hpp"
//#include "95071e/pid.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace team_e {

class Backpack {
 public:
  Backpack() {}

  double get_rotation_position() {
    return static_cast<double>(lift_sensor_.get_position()) / 100.0;
  }

  /*void move_with_pid(double target_position) {
    constexpr double position_tolerance = 1.0;
    lift_controller.reset();

    double previous_time = static_cast<double>(pros::c::micros()) * 1.0e-6;
    double previous_position = get_rotation_position();

    lift_profiler.reset();
    lift_profiler.set_initial_position(previous_position);
    lift_profiler.set_target_postition(target_position);
    lift_profiler.start();
    while (true) {
      pros::delay(10);
      const double current_time =
          static_cast<double>(pros::c::micros()) * 1.0e-6;
      const double position = get_rotation_position();
      const double dt = current_time - previous_time;
      const double velocity = lift.get_actual_velocity() * 0.8 / 60.0 * 360.0;
      MotionProfiler::MotionState target_state = lift_profiler.run(dt);

      pros::lcd::print(1, "target position: %f", target_state.position);
      pros::lcd::print(3, "target velocity: %f", target_state.velocity);

      const double lift_ratio_voltage_ = 0.02 * 60.0;
      const double lift_feedforward = 0;
      const double position_error = target_position - position;
      const double velocity_error = target_state.velocity - velocity;
      double lift_control =
          lift_feedforward +
          lift_controller.get_control(dt, position_error, std::nullopt);

      pros::lcd::print(2, "Actual position: %f", position);
      pros::lcd::print(4, "Actual velocity: %f", velocity);

      lift_control = std::max(-0.5 * max_control_rpm_,
                              std::min(max_control_rpm_, lift_control));

      pros::lcd::print(5, "Control: %f", lift_control);

      lift.move_velocity(lift_control);
      if (std::abs(position - target_position) < position_tolerance) {
        lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        lift.move_velocity(0);
        break;
      }
      previous_position = position;
      previous_time = current_time;
    }
  }*/

  void move(double target_position, int speed) {
    double position = get_rotation_position();
    double position_tolerance = 3.0;
    double moving_speed = 0;
    if (target_position - position > position_tolerance) {
      moving_speed = speed;
    } else if (target_position - position < -position_tolerance) {
      moving_speed = -speed;
    }
    while (true) {
      lift.move_velocity(moving_speed);
      position = get_rotation_position();
      //pros::lcd::print(1, "target position: %f", target_position);
      //pros::lcd::print(2, "current position: %f", position);
      if (std::abs(position - target_position) < position_tolerance) {
        lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        lift.move_velocity(0);
        break;
      }
      pros::delay(10);
    }
  }

  void forward_a_stage() {
    switch (stage_) {
      case Stage::DOWN:
        // lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
        move(18.0, 60);
        stage_ = Stage::LOAD;
        break;
      case Stage::LOAD:
        move(up_position_, 200);
        stage_ = Stage::UP;
        break;
      case Stage::UP:
        // lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        move(down_position_, 140);
        stage_ = Stage::DOWN;
        break;
    }
  }

  void back_a_stage() {
    switch (stage_) {
      case Stage::DOWN:
        break;
      case Stage::LOAD:
        // lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        move(2.0, 40);
        stage_ = Stage::DOWN;
        break;
      case Stage::UP:
        // lift.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
        move(23.0, 70);
        stage_ = Stage::LOAD;
        break;
    }
  }

 private:
  enum class Stage { DOWN, LOAD, UP };

  Stage stage_ = Stage::DOWN;

  static constexpr double down_position_ = 2.0;
  static constexpr double load_position_ = 30.0;
  static constexpr double up_position_ = 140.0;

  static constexpr double lift_max_speed = 90.0;
  static constexpr double lift_max_acceleration = lift_max_speed / 2.0;
  static constexpr double max_motor_rpm_ = 100.0;  // max allowed by API.
  static constexpr double max_control_rpm_ = max_motor_rpm_ * 0.8;

  //PIDControl lift_controller{1.0, 0.0, 0.1};

  //team_e::MotionProfiler lift_profiler{lift_max_speed, -lift_max_speed,
  //                                     lift_max_acceleration,
  //                                     0.5 * lift_max_acceleration};

  pros::Motor lift{8, pros::v5::MotorGear::red, pros::v5::MotorUnits::degrees};
  pros::Rotation lift_sensor_{9};
};

}  // namespace team_e

#endif  // BACKPACK_HPP
