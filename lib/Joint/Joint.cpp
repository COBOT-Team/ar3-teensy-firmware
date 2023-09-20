#include "Joint.h"

Joint::Joint(JointConfig config)
  : config(config)
  , stepper(AccelStepper::DRIVER, config.step_pin, config.dir_pin)
  , encoder(config.enc_a_pin, config.enc_b_pin)
  , limit_switch(config.lim_pin)
{
  this->state.id = State::IDLE;
  this->is_calibrated = false;
  this->micros_timer = 0;
  this->print_timer = 0;

  this->enc_deg_per_tick = 360.0 / ((float)config.enc_ticks_per_rev * config.enc_reduction);
  this->motor_deg_per_step = 360.0 / ((float)config.motor_steps_per_rev * config.motor_reduction);
  this->enc_ticks_per_step = (float)config.enc_ticks_per_rev / (float)config.motor_steps_per_rev;

  this->stepper.setMinPulseWidth(5);  // VERY IMPORTANT, THIS TOOK FUCKING HOURS TO FIGURE OUT
  this->stepper.setMaxSpeed(config.max_speed / motor_deg_per_step);
  this->stepper.setAcceleration(config.max_accel / motor_deg_per_step);
}

void Joint::init()
{
  pinMode(config.lim_pin, INPUT_PULLDOWN);
}

Joint::State* Joint::get_state()
{
  return &state;
}

float Joint::get_position()
{
  return encoder.read() * enc_deg_per_tick;
}

float Joint::get_speed()
{
  return measured_speed;
}

void Joint::move_to_auto(int32_t target)
{
  float target_f = target * 0.001f;
  state.id = State::MOVE_TO_AUTO;
  state.data.move_to_auto.target_steps = config.direction * target_f / motor_deg_per_step;

  // Limit the target position to the range of the joint.
  if (state.data.move_to_auto.target_steps < config.min_steps)
    state.data.move_to_auto.target_steps = config.min_steps;
  if (state.data.move_to_auto.target_steps > config.max_steps)
    state.data.move_to_auto.target_steps = config.max_steps;

  stepper.moveTo(state.data.move_to_auto.target_steps);
}

void Joint::move_to_speed(int32_t target, int32_t speed)
{
  float target_f = target * 0.001f;
  state.id = State::MOVE_TO_SPEED;
  state.data.move_to_speed.target_steps = config.direction * target_f / motor_deg_per_step;
  state.data.move_to_speed.speed = speed / motor_deg_per_step;

  // Limit the target position to the range of the joint.
  if (state.data.move_to_speed.target_steps < config.min_steps)
    state.data.move_to_speed.target_steps = config.min_steps;
  if (state.data.move_to_speed.target_steps > config.max_steps)
    state.data.move_to_speed.target_steps = config.max_steps;

  // Limit the speed to between 0 and the maximum speed of the joint.
  if (state.data.move_to_speed.speed < 0) state.data.move_to_speed.speed = 0;
  if (state.data.move_to_speed.speed > config.max_speed)
    state.data.move_to_speed.speed = config.max_speed;

  stepper.moveTo(state.data.move_to_speed.target_steps);
  stepper.setSpeed(state.data.move_to_speed.speed);
}

void Joint::move_forever_speed(int32_t speed)
{
  float speed_f = speed * 0.001f;
  state.id = State::MOVE_FOREVER_SPEED;
  state.data.move_forever_speed.speed = config.direction * speed_f / motor_deg_per_step;

  // Limit the speed to between -max_speed and max_speed.
  if (state.data.move_forever_speed.speed < -config.max_speed)
    state.data.move_forever_speed.speed = -config.max_speed;
  if (state.data.move_forever_speed.speed > config.max_speed)
    state.data.move_forever_speed.speed = config.max_speed;

  stepper.setSpeed(state.data.move_forever_speed.speed);
}

void Joint::stop(bool smooth)
{
  if (smooth && is_calibrated) {
    state.id = State::STOPPING;
    stepper.stop();
  } else {
    state.id = State::IDLE;
    stepper.setSpeed(0);
  }
}

void Joint::calibrate()
{
  state.id = State::CALIBRATING;
  state.data.calibrate.has_hit_limit_switch = false;
  stepper.setSpeed(config.calibration_speed / motor_deg_per_step);
}

bool Joint::get_is_calibrated()
{
  return is_calibrated;
}

void Joint::override_position(int32_t position)
{
  float position_f = position * 0.001f;
  state.id = State::IDLE;
  is_calibrated = true;
  stepper.setCurrentPosition(position_f / motor_deg_per_step);
}

bool Joint::position_within_range(int32_t position)
{
  float position_f = position * 0.001f;
  return (position_f >= config.min_steps * motor_deg_per_step) &&
         (position_f <= config.max_steps * motor_deg_per_step);
}

bool Joint::speed_within_range(int32_t speed)
{
  float speed_f = speed * 0.001f;
  return (speed_f >= -config.max_speed) && (speed_f <= config.max_speed);
}

void Joint::reset()
{
  state.id = State::IDLE;
  is_calibrated = false;
  stepper.setSpeed(0);
  stepper.runSpeed();
  stepper.setCurrentPosition(0);
  encoder.write(0);
  last_encoder_pos = 0;
  measured_speed = 0;
  micros_timer = 0;
}

void Joint::update()
{
  switch (state.id) {
    case State::IDLE:
      break;

    case State::STOPPING:
      if (!stepper.isRunning()) {
        state.id = State::IDLE;
        stepper.setSpeed(0);
        break;
      }
      stepper.run();
      break;

    case State::CALIBRATING: {
      if (state.data.calibrate.has_hit_limit_switch) {
        if (!stepper.isRunning()) {
          state.id = State::IDLE;
          is_calibrated = true;
          stepper.setSpeed(0);
          break;
        }
        stepper.run();
      } else {
        if (limit_switch.read() && limit_switch.read_interval(1, 100)) {
          stepper.setCurrentPosition(config.ref_steps);
          encoder.write(config.ref_steps * enc_ticks_per_step);
          stepper.moveTo(config.goto_after_calibrate);
          state.data.calibrate.has_hit_limit_switch = true;
        }
        stepper.runSpeed();
      }

    } break;

    case State::MOVE_TO_AUTO:
      if (!stepper.isRunning()) {
        state.id = State::IDLE;
        stepper.setSpeed(0);
        break;
      }
      stepper.run();
      break;

    case State::MOVE_TO_SPEED:
      if (stepper.distanceToGo() == 0) {
        state.id = State::IDLE;
        stepper.setSpeed(0);
      }
      stepper.runSpeedToPosition();
      break;

    case State::MOVE_FOREVER_SPEED:
      if (stepper.currentPosition() <= config.min_steps ||
          stepper.currentPosition() >= config.max_steps) {
        state.id = State::IDLE;
        stepper.setSpeed(0);
      }
      stepper.runSpeed();
      break;
  }

  /*
   * Calculate the speed of the joint. This uses a moving mean filter, scaled by the time since the
   * last update. This way, the filter is independent of the update rate.
   */

  int32_t encoder_pos = encoder.read();
  float dt = micros_timer / 1000000.0f;
  if (dt == 0) return;
  micros_timer = 0;

  float unfiltered_speed = (encoder_pos - last_encoder_pos) * enc_deg_per_tick / dt;
  float scaled_filter_strength = config.speed_filter_strength * dt;

  if (scaled_filter_strength >= 1) {
    measured_speed = unfiltered_speed;
  } else {
    measured_speed =
      measured_speed * (1 - scaled_filter_strength) + unfiltered_speed * scaled_filter_strength;
  }
  last_encoder_pos = encoder_pos;

  // if (print_timer > 100) {
  //   print_timer = 0;
  //   if (config.id == 5) {
  //     Serial.print("j ");
  //     Serial.print(config.id);
  //     Serial.print(", s ");
  //     Serial.print(stepper.currentPosition());
  //     Serial.print(", e ");
  //     Serial.print(encoder_pos);
  //     Serial.print(", r ");
  //     Serial.println(enc_ticks_per_step);
  //   }
  // }
}