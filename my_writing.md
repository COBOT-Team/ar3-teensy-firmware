# Existing resources

This section will explain the main software and hardware components that were inherited from previous projects. It will discuss which elements we used, which ones we did not, and our reasoning for these decisions.

## AR3 Robot Arms

Two Annin Robotics AR3 robot arms were built by previous projects. There are numerous differences between these, which will be discussed in the Project Architecture section. The easiest way to distinguish between the two robots is by their color scheme: one has purple accents while the other has red.

TODO: add images

These robots ran firmware developed by Annin Robotics. Unfortunately, this firmware was difficult to work with, disorganized, and generally poorly written. An apparent aversion to arrays and functions leads to hundreds of lines of repeated code that make maintenance difficult. It also appears to rely on external software to account for joint limits and basic safety features. For these reasons, we determined that developing a custom firmware was prudent.

Both robots suffer from unreliability (to differing degrees). Certain joints, especially the elbows, have significant mechanical backlash that makes it difficult to determine the true angle of the joints. This issue is more pronounced in the purple robot.

Because the robots do not have absolute encoders on their joints, a calibration procedure must be run every time they are turned on. Limit switches on each joint are used to identify when the joints reach certain known angles, which are used as reference points for all future movement. This calibration is not always completely reliable due to various factors, including mechanical inconsistencies in the joints and the bending of limit switches over time. The quadrature encoders on the stepper motors have index pins which could be used to mitigate this, but these are not currently wired into the microcontrollers.

These issues affect each robot to a different extent. They are covered in more detail in the Project Architecture section.

## ROS Code

In addition to physical robots, we received ROS code from the previous team. That team approached their project very differently than we planned to; they created a custom chessboard and chess pieces in order to avoid any computer vision and use an off-the-shelf VEX Robotics gripper. All positions were hard-coded and manual assistance was necessary when moving joints due to physical and electrical limitations of the purple robot (see Project Architecture).

TODO: images of their old claw, chessboard, and chess pieces

Additionally, their software stack was based on ROS1, while we wanted our project to use ROS2. Finally, a new hardware interface was required to interact with the custom firmware we planned to write for the robots.

For these reasons, we determined that very little code could be easily carried over into our project. Instead, we wrote all of our software from scratch.

# Tasks

## New AR3 Firmware

To address the concerns raised in Existing Resources, we developed a new firmware for the AR3 robots with three main goals: clean and extensible code, inbuilt safety features, and fast real-time communication with the main computer.

### Code Quality

The overall architecture of the code will be covered in the Implementation Details section. In brief, we wanted this firmware to be easily adapted to robot arms with (slightly) differing hardware characteristics and end effectors. We also aimed to write code that was easy to understand and well-documented. Additionally, we implemented full-featured logging to aid in debugging.

### Safety Features

We believe it important that every layer of abstraction in a software system implements appropriate safety features. Any individual software component should be expected to fail, and all other components should be designed to accommodate these failures as safely and gracefully as possible. The Annin Robotics firmware relied on external software to account for even the most basic of safety features, including ensuring that joints do not move beyond their physical limits.

Our firmware was developed with safety as the foremost consideration. Physical limits, including velocity, are imposed at the firmware level. Joints cannot be moved without being properly calibrated, and a safe calibration order is strictly enforced to ensure joints do not collide. In short, we set out to create a firmware that would catch and account for any error or safety issue that it could possibly have knowledge of.

### Fast Communication

The AR3 communicates over USB via a virtual serial adapter. The previous firmware used a relatively slow text-based protocol. While this is useful for manually sending and receiving commands, we ultimately determined that a more robust, faster binary protocol was better.

The robot must communicate with the controlling computer at high speeds, sending state updates at up to 100Hz. It must also control 6 stepper motors in real-time, all of which require precise timing. While the Teensy microcontrollers do implement interrupt-driven serial communication, frequent I/O still wastes clock cycles and defers encoder ISRs. We determined that this, in addition to the additional time required to parse, process, and send ASCII messages, could potentially cause hard-to-debug synchronization issues. A smaller binary protocol would be less susceptible to this.

## ros2_control and Hardware Interface

One major limitation of the DDS messaging system that ROS2 uses is speed. Typically, messages are sent over standard network interfaces (e.g. TCP/IP), which has non-negligible latency. Even when using shared memory via ROS2 Components, the overhead of the pub/sub system can be problematic for real-time systems. The ros2_control framework solves this problem by dynamically loading hardware interfaces at runtime. This allows real-time controllers to interact with hardware in a generic way.

MoveIt2 typically uses ros2_control to interface with hardware. Therefore, we needed to create a hardware interface for our new AR3 firmware. More details on this interface can be found in the Implementation Details section.

## URDF and MoveIt2 Config

The Universal Robot Description Format (URDF) is used throughout ROS to define the physical properties of a robot, including its appearance, collision meshes, and joint properties. Both ros2_control and MoveIt2 require a robot description in this format in order to properly load the robot’s joints and, in the case of MoveIt2, generate valid motion profiles. While the previous team did have a URDF description for the AR3, ROS2 necessitates differences in the format of the ros2_control and MoveIt2 sections. We also wanted the experience of designing a proper robot description file. For these reasons, we decided to write a new URDF.

We also needed a MoveIt2 configuration for the AR3s so that it can properly plan motion. This was mostly generated via the convenient Setup Assistant, but some changes had to be made to integrate our hardware interface. See Implementation Details for more information.

## Gripper

The gripper was one of the most important elements of this project. We were not satisfied with any off-the-shelf grippers that we could find. Instead, we designed our own with the following constraints:

### Precise Manipulation

Chess pieces are small and they can be very close together. To ensure the gripper is never blocked or inhibited by improperly placed pieces, we determined that it must be able to successfully pick up a pawn surrounded on all sides by kings. This is the worst-case scenario that is possible on a chessboard, and a gripper that can do this should never have any issues picking up or placing pieces.

## Embedded Camera

Early on, we predicted that we may have issues with the precision of the robot arms and our scene tracking. Therefore, we determined that it would be prudent to design a gripper with space for a depth camera to be embedded in the center. This would be used for visual servoing to ensure pieces are properly centered in the gripper before being picked up.

## Clock

Within the scope of this project, one of the pivotal tasks was the development of a chess clock conducive to robotic interaction. The goal was to engineer a chess clock, utilizing 3D printing technology for precise fabrication, that can seamlessly integrate with our robotic system. This clock is designed with an extended lever, which the robot is programmed to depress once it completes its move, signifying the transition of the turn to the human opponent. A USB interface is implemented for communication, facilitating the exchange of signals between the robot and the clock mechanism to accurately track and manage the time allocation for each player. The image below illustrates the prototype of our custom-designed chess clock, marked with the inscription 'Cobot 2024', denoting the year of the project's conception. 

## Camera Nodes

In addition to the AR3 robots, we needed ROS2 to interact with two other hardware components: an overhead camera and the depth cameras embedded in the robots’ grippers. We used a Kinect for XBox One for the overhead camera due to its depth camera, and because we already had four of them in the lab. We did not find a satisfactory ROS2 node for this Kinect model, so we wrote our own. We used ArduCam time-of-flight cameras in the grippers. ArduCam does provide a ROS2 node for this model, but it publishes a pointcloud that we found to be unreliable at close range. Instead, we opted to publish raw depth images that could be processed with conventional computer vision techniques. We therefore had to write our own node for this as well.

Accurate calibration was required for these cameras, especially the Kinect, to correct for distortion and ensure pose estimates are accurate. More information can be found in Implementation Details.

## Scene Tracking

In order to be as dynamic and reliable as possible, we wanted to actively track the locations of important objects. At minimum, we set out to track the relative locations of the table and the chessboard. The robots must be bolted to the table, so they do not need to be tracked independently; a static transformation from the table is sufficient.

We determined that the best way to achieve this tracking is with fiducial markers. We chose to use ArUco since that functionality is built into OpenCV, but we planned to experiment with STag if time permitted. It did not.

## Chess Engine

While we initially hoped to develop a custom chess engine, it became quickly apparent that this was unrealistic. Instead, we needed to write a ROS2 node that would launch and manage chess engines that implemented the Universal Chess Interface (UCI). The most famous of these is Stockfish, an open-source chess engine that is widely considered to be the best chess player in the world. We used Stockfish in this project, but we made sure to design our ROS2 node such that the engine could be easily swapped with a parameter.

## Chess-Playing Code

The most obvious piece of software, this ROS2 node must watch the game state and, when applicable, move the robot arm to take its turn. It must also press the chess clock to signal the end of its turn.

It was very important that this code implemented the following features:

### Safety

Safety is always the top priority, and this code is no exception. The most important consideration was ensuring that the robot would properly disable itself when commanded to, regardless of its current state. In code like this, it is very easy for processes to become out-of-sync. For example, the robot may be disabled while this code is planning its next motion. If the state is not properly checked, we may then command the robot to execute this motion despite it being disabled.

### Visual Servoing

Precise alignment to small objects can be difficult, particularly with the backlash and calibration issues of the AR3. To improve this, the depth camera embedded in the gripper can be used to perform visual servoing: closed-loop cartesian motion of the gripper that centers it on top of the desired chess piece. It was vital that this be implemented, as pieces could not be picked up reliably without it.

### Safe Clock Interaction

To signal the end of its turn, the AR3 must press a button on the chess clock. This requires millimeter precision; if the AR3 attempts to move slightly too low, it will press itself into the clock, potentially damaging components. To achieve safe interaction, we determined that servoing was again necessary. The AR3 servos down in the Z axis until it receives feedback from the clock that it has pressed the button. It then immediately stops moving to avoid damage.

## Backlash Compensation

As we have mentioned previously, the AR3 is plagued by backlash issues. The purple one in particular has wildly inconsistent motion because of this. In the interest of using both robots to play, we set out to compensate for this backlash as much as possible. We attempted two major software solutions: visual compensation and absolute joint measurement. These are covered in the Implementation Details section.

Unfortunately, our software solutions were insufficient, and we were not able to get the purple AR3 working reliably.

# Implementation Details

## AR3 Firmware

As mentioned in Tasks, this firmware was designed to be high quality, well-documented, safe, and fast.

### Software Framework
The firmware is written with PlatformIO, using the Ardunio framework. This allows it to access Arduino-specific libraries  and provides convenient build configuration. We can take advantage of the fact that our two AR3s use two different microcontrollers by defining build flags based on the target board. This allows code to know which AR3 it is being compiled for, and it can load different configurations based on this.

```
[env:cobot0]
platform = teensy
board = teensy41
framework = arduino
lib_deps =
    waspinator/AccelStepper@^1.64
    paulstoffregen/Encoder@^1.4.2
    paulstoffregen/PWMServo@^2.1
build_flags = -D COBOT_ID=0


[env:cobot1]
platform = teensy
board = teensy35
framework = arduino
lib_deps =
    waspinator/AccelStepper@^1.64
    paulstoffregen/Encoder@^1.4.2
    paulstoffregen/PWMServo@^2.1
build_flags =
    -D COBOT_ID=1
    -llibc -lc
```

### Configuration

Our two AR3 robots had differences in many of their joints, which required different firmware configurations. Pins, joint directions, and joint limits are different between robots. To accommodate this, a Joint class and a Gripper class were created. These store all of the relevant information for specific joints and grippers, and are initialized with JointConfig and GripperConfig structs. Different robot configurations are defined in different source files and enabled conditionally based on the COBOT_ID compiler flag.

```cpp
struct JointConfig {
  uint8_t id;        // ID of the joint (0-5)
  const char* name;  // Name of the joint

  int32_t min_steps;  // Most negative position of the joint (in steps)
  int32_t max_steps;  // Most positive position of the joint (in steps)
  int32_t ref_steps;  // Position of the joint when it touches the limit switch (in steps)

  int32_t goto_after_calibrate;  // Position of the joint to go to after calibration (in steps).
                                 // This is included in the joint's calibration procedure.

  int motor_steps_per_rev;  // Number of steps per revolution of the stepper motor
  int enc_ticks_per_rev;    // Number of ticks per revolution of the encoder

  float motor_reduction;  // Gear reduction from the motor to the joint
  float enc_reduction;    // Gear reduction from the encoder to the joint

  int8_t direction;  // 1 if the joint is not reversed, -1 if it is reversed

  float max_speed;  // Maximum speed of the joint (in degrees per second)
  float max_accel;  // Maximum acceleration of the joint (in degrees per second per second)

  float calibration_speed;  // Speed of the joint during calibration (in degrees per second). Sign
                            // determines the direction of calibration. `direction` is ignored
                            // during calibration.

  uint8_t step_pin;  // Step pin of the motor controller
  uint8_t dir_pin;   // Direction pin of the motor controller

  uint8_t enc_a_pin;  // Encoder A pin
  uint8_t enc_b_pin;  // Encoder B pin

  float speed_filter_strength;  // Strength of the speed filter (percent of new speed per second)

  uint8_t lim_pin;  // Limit switch pin
};
```

```cpp
struct GripperConfig {
  int pin;           // The pin that the gripper's servo is attached to
  int min_angle;     // The minimum angle (degrees) that the servo is allowed to go to.
  int max_angle;     // The maximum angle (degrees) that the server is allowed to go to.
  double avg_speed;  // The average speed of the gripper servo (deg/sec).
};
```

```cpp
#if COBOT_ID == 1

#include "config.h"

Gripper gripper({
  .pin = 23,
  .min_angle = 0,
  .max_angle = 180,
  .avg_speed = 450,
});

// clang-format off

Joint joints[] = {
  Joint({
    .id = 0,
    .name = "base",


    .min_steps = -8000,
    .max_steps = 6000,
    .ref_steps = -8000,
   
    .goto_after_calibrate = 0,


    .motor_steps_per_rev = 400,
    .enc_ticks_per_rev = 2048,


    .motor_reduction = 40.0,
    .enc_reduction = 40.0,


    .direction = -1,


    .max_speed = 80.0,
    .max_accel = 240.0,
    .calibration_speed = -10.0,


    .step_pin = 0,
    .dir_pin = 1,
    .enc_a_pin = 14,
    .enc_b_pin = 15,


    .speed_filter_strength = 5.0,


    .lim_pin = 26,
  }),
 
  Joint({
    .id = 1,
    .name = "shoulder",


    .min_steps = -5000,
    .max_steps = 2400,
    .ref_steps = 2400,
   
    .goto_after_calibrate = 1750,


    .motor_steps_per_rev = 400,
    .enc_ticks_per_rev = 2048,


    .motor_reduction = 50.0,
    .enc_reduction = 50.0,


    .direction = -1,


    .max_speed = 60.0,
    .max_accel = 240.0,
    .calibration_speed = 5.0,


    .step_pin = 2,
    .dir_pin = 3,
    .enc_a_pin = 17,
    .enc_b_pin = 16,


    .speed_filter_strength = 5.0,


    .lim_pin = 27,
  }),
 
  Joint({
    .id = 2,
    .name = "elbow",


    .min_steps = -8200,
    .max_steps = 5000,
    .ref_steps = -8200,
   
    .goto_after_calibrate = -3500,

    .motor_steps_per_rev = 400,
    .enc_ticks_per_rev = 2048,

    .motor_reduction = 50.0,
    .enc_reduction = 50.0,

    .direction = -1,

    .max_speed = 160.0,
    .max_accel = 220.0,
    .calibration_speed = -5.0,

    .step_pin = 4,
    .dir_pin = 5,
    .enc_a_pin = 19,
    .enc_b_pin = 18,

    .speed_filter_strength = 5.0,

    .lim_pin = 28,
  }),
 
  Joint({
    .id = 3,
    .name = "forearm roll",

    .min_steps = -7400,
    .max_steps = 4500,
    .ref_steps = -7400,
   
    .goto_after_calibrate = 0,

    .motor_steps_per_rev = 400,
    .enc_ticks_per_rev = 2048,

    .motor_reduction = 39.2,
    .enc_reduction = 39.2,

    .direction = 1,

    .max_speed = 240.0,
    .max_accel = 1200.0,
    .calibration_speed = -10.0,

    .step_pin = 6,
    .dir_pin = 7,
    .enc_a_pin = 33,
    .enc_b_pin = 34,

    .speed_filter_strength = 5.0,

    .lim_pin = 29,
  }),
 
  Joint({
    .id = 4,
    .name = "wrist pitch",


    .min_steps = -2100,
    .max_steps = 2100,
    .ref_steps = -2100,
   
    .goto_after_calibrate = 0,


    .motor_steps_per_rev = 800,
    .enc_ticks_per_rev = 2048,


    .motor_reduction = 9.1455,
    .enc_reduction = 9.1455,


    .direction = 1,


    .max_speed = 200.0,
    .max_accel = 600.0,
    .calibration_speed = -10.0,


    .step_pin = 8,
    .dir_pin = 9,
    .enc_a_pin = 35,
    .enc_b_pin = 36,


    .speed_filter_strength = 5.0,


    .lim_pin = 30,
  }),
 
  Joint({
    .id = 5,
    .name = "wrist roll",


    .min_steps = -3588,
    .max_steps = 3700,
    .ref_steps = -3285,
   
    .goto_after_calibrate = 0,


    .motor_steps_per_rev = 400,
    .enc_ticks_per_rev = 2048,


    .motor_reduction = 19.0,
    .enc_reduction = 19.0,


    .direction = -1,


    .max_speed = 1500.0,
    .max_accel = 8000.0,
    .calibration_speed = -10.0,


    .step_pin = 10,
    .dir_pin = 11,
    .enc_a_pin = 38,
    .enc_b_pin = 37,


    .speed_filter_strength = 5.0,


    .lim_pin = 31,
  })
};


// clang-format on


#endif
```

### Architecture

The firmware has three main components: the messenger, the joint controllers, and the coordinator. The messenger handles all communication between the host computer and the firmware. It frames messages, verifies CRCs, handles ACKs, and has high-level logging functions. Joint controllers interface with a single joint. These are instances of the Joint or Gripper class and operate via internal state machines. Joints are responsible for most safety features, as they are the final authority on when and how all motion is executed. Finally, the central coordinator sends and receives messages to the messenger and commands joints to act. Like joints, it operates via a state machine.

#### Messaging

<!-- In other file -->

#### Joint Managers

Each joint is managed with an independent state machine inside the Joint class. The AR3 has an array of ordered Joints that it uses to command and update them. The joint can be in one of the following states:

State               Behavior
IDLE                The joint is not moving
STOPPING            The joint is smoothly stopping
CALIBRATING         The joint is calibrating
MOVE_TO_AUTO        The joint is moving to a target position with an automatically calculated speed
MOVE_TO_SPEED       The joint is moving to a target position at a specific speed
MOVE_FOREVER_SPEED  The joint is moving indefinitely at a specific speed

Some states have data associated with them. For instance, a joint in the CALIBRATING state keeps track of whether or not it has hit its limit switch.

```cpp
struct State {
  // The ID of the state.
  enum {
    IDLE,           // The joint is not moving
    STOPPING,       // The joint is stopping
    CALIBRATING,    // The joint is calibrating
    MOVE_TO_AUTO,   // The joint is moving to a position with an automatically calculated speed
    MOVE_TO_SPEED,  // The joint is moving to a position with a specified speed
    MOVE_FOREVER_SPEED,  // The joint is moving indefinitely at a specified speed
  } id;


  // Data dependent on the state.
  union {
    struct {
      bool has_hit_limit_switch;  // Whether or not the joint has hit the limit switch
    } calibrate;


    struct {
      int32_t target_steps;  // The target position of the joint (in steps)
    } move_to_auto;


    struct {
      int32_t target_steps;  // The target position of the joint (in steps)
      float speed;           // The speed of the joint (in steps per second)
    } move_to_speed;


    struct {
      float speed;  // The speed of the joint (in steps per second)
    } move_forever_speed;
  } data;
};```

Public methods are used to update the state of the Joint. Functionality associated with the state is executed in the update() method.

```cpp
void Joint::move_to_auto(int32_t target)
{
  if (encoder_feedback_enabled) fix_stepper_position();


  float target_f = target * 0.001f;
  state.id = State::MOVE_TO_AUTO;
  state.data.move_to_auto.target_steps = config.direction * target_f / motor_deg_per_step;


  // Limit the target position to the range of the joint.
  if (state.data.move_to_auto.target_steps < config.min_steps)
    state.data.move_to_auto.target_steps = config.min_steps;
  if (state.data.move_to_auto.target_steps > config.max_steps)
    state.data.move_to_auto.target_steps = config.max_steps;


  stepper.moveTo(state.data.move_to_auto.target_steps);
}```

```cpp
void Joint::update()
{
  switch (state.id) {
    case State::IDLE:
      break;


    case State::STOPPING:
      if (stepper.speed() == 0) {
        if (encoder_feedback_enabled) fix_stepper_position();
        state.id = State::IDLE;
        break;
      }
      stepper.run();
      break;


    case State::CALIBRATING: {
      if (state.data.calibrate.has_hit_limit_switch) {
        if (!stepper.isRunning()) {
          if (encoder_feedback_enabled) fix_stepper_position();
          state.id = State::IDLE;
          is_calibrated = true;
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
        if (encoder_feedback_enabled) fix_stepper_position();
        state.id = State::IDLE;
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
}
```

#### Gripper Manager

The Gripper class is extremely simple; it simply wraps a PWM-controller servo motor. The only notable feature is its position estimation. MoveIt2 requires a position state interface - essentially, a position measurement. Because the servo does not give feedback, and adding an external potentiometer would be undesirable, we estimate the servo’s position in the firmware based on its average velocity. This measurement does not need to be particularly accurate, so characteristics such as acceleration and load are not taken into account.

```cpp
void Gripper::move_to(int dest)
{
  if (dest < config.min_angle) dest = config.min_angle;
  if (dest > config.max_angle) dest = config.max_angle;


  last_pos_ = get_position_estimate();
  set_pos_ = dest;
  move_speed_ = (dest > last_pos_) ? config.avg_speed : -config.avg_speed;
  pos_interpolation_timer = 0;
  servo_.write(dest);
}

int Gripper::get_position_estimate()
{
  const int error = set_pos_ - last_pos_;
  if (error == 0) return set_pos_;


  const float travelled = move_speed_ * pos_interpolation_timer * 0.001;
  const int travelled_int = static_cast<int>(travelled);
  if (abs(travelled_int) >= abs(error)) {
    last_pos_ = set_pos_;
    move_speed_ = 0;
    return set_pos_;
  }


  return last_pos_ + travelled_int;
}
```

#### Coordinator

The coordinator is the equivalent of the firmware’s “main” function. It uses the Arduino framework, initializing joints in setup() and updating them in loop(). Serial data is also read and parsed in loop(). Because stepper motors must be pulsed at precise intervals, it is extremely important that loop() does not impose any delays. Reading from the serial port is one of the most sensitive operations in this respect; if done improperly, this may take too long and de-sync stepper motors. Fortunately, the Teensy uses interrupt-driven buffered I/O, so reading available data is fast. More importantly, writing is asynchronous. Data is placed directly into the USB buffer, which is managed by dedicated hardware.

```cpp
// Read from the serial port until the buffer is full or there are no more bytes to read.
if (!Serial.available()) return;
while (serial_buffer_in_len < SERIAL_BUFFER_SIZE) {
  int x = Serial.read();
  if (x == -1) break;
  if (x == START_BYTE) message_in_progress = true;
  if (message_in_progress) serial_buffer_in[serial_buffer_in_len++] = x;
}

// Try parsing a message.
int msg_len = framing::check_message(serial_buffer_in, serial_buffer_in_len);
if (msg_len == 0) return;  // Message is incomplete


// If the message is invalid, send an error log message and discard it.
if (msg_len == -1) {
  messenger.log(LogLevel::WARN, "Invalid message received.");
  serial_buffer_in_len = framing::remove_bad_frame(serial_buffer_in, serial_buffer_in_len);
  return;
}

// Parse the message as a request.
if (msg_len < 5) {
  messenger.log(LogLevel::WARN, "Message is too short to be a request.");
  serial_buffer_in_len = framing::remove_bad_frame(serial_buffer_in, serial_buffer_in_len);
  return;
}
uint8_t msg_payload_len = msg_len - 5;
const uint8_t* msg = serial_buffer_in + (serial_buffer_in_len - msg_len);
uint8_t request_type = msg[0];
int32_t request_id;
deserialize_int32(&request_id, msg + 1);
```

One factor of note is that the USB output buffer does not immediately send data; it instead waits a few milliseconds in case additional data is sent. (https://www.pjrc.com/teensy/td_serial.html#txbuffer) This is done to reduce the number of individual USB packets being sent, but it has the side effect of slightly increasing the latency of serial communication. While this did not appear to cause any issues, it may still be desirable to force time-sensitive data to be sent immediately with Serial.send_now(). This was not implemented in our firmware, but could be easily added.

Once a message is fully received, it is processed by a request handler, which takes the form of a function.

```cpp
// Handle the request.
messenger.log(LogLevel::DEBUG, "Handling request %u (%s)", request_id,
              messenger.get_request_name(request_type));
switch (request_type) {
  case static_cast<uint8_t>(Request::Init):
    handle_init(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::Calibrate):
    handle_calibrate(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::Override):
    handle_override(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::GetJoints):
    handle_get_joints(request_id);
    break;
  case static_cast<uint8_t>(Request::MoveTo):
    handle_move_to(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::MoveSpeed):
    handle_move_speed(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::FollowTrajectory):
    handle_follow_trajectory(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::Stop):
    handle_stop(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::GoHome):
    handle_go_home(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::Reset):
    handle_reset(request_id);
    return;
  case static_cast<uint8_t>(Request::SetLogLevel):
    handle_set_log_level(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::SetFeedback):
    handle_set_feedback(request_id, msg + 5, msg_payload_len);
    break;
  case static_cast<uint8_t>(Request::SetGripper):
    handle_set_gripper(request_id, msg + 5, msg_payload_len);
    break;


  default:
    messenger.send_error_response(request_id, ErrorCode::MALFORMED_REQUEST,
                                  "Unknown request type");
    break;
}
```

```cpp
/**
 * Handles an Init request.
 *
 * @param[in] request_id The ID of the request.
 * @param[in] data The request data.
 * @param[in] data_len The length of the data buffer.
 */
void handle_init(uint32_t request_id, const uint8_t* data, uint8_t data_len)
{
  if (data_len != sizeof(uint32_t)) {
    return messenger.send_error_response(request_id, ErrorCode::MALFORMED_REQUEST,
                                         "(INIT) Payload is the wrong size (expected %u, got "
                                         "%u)",
                                         sizeof(uint32_t), data_len);
  }


  uint32_t expected_fw_version;
  deserialize_uint32(&expected_fw_version, data);


  if (expected_fw_version == FW_VERSION) {
    initialized = true;
    messenger.log(LogLevel::INFO, "Initialized (firmware version %lu)", FW_VERSION);
    return messenger.send_ack(request_id);
  } else {
    return messenger.send_error_response(request_id, ErrorCode::INVALID_FIRMWARE_VERSION,
                                         "(INIT) COBOT is running firmware version %lu, "
                                         "expected version %lu",
                                         FW_VERSION, expected_fw_version);
  }
}
```

The coordinator implements a state machine that controls the overall robot state. This is separate from the states of the individual joints.

```cpp
/**
 * A state of the cobot. This is comprised of a state ID and a message ID. The message ID is used to
 * send responses to the message that initiated the state. If the state is IDLE, the message ID is
 * ignored.
 */
struct CobotState {
  // The ID of the state.
  enum {
    IDLE,               // The cobot is not moving
    STOPPING,           // The cobot is stopping
    CALIBRATING,        // The cobot is calibrating
    MOVE_TO,            // The cobot is moving to a specified position
    MOVE_SPEED,         // The cobot is moving indefinitely at a specified speed
    FOLLOW_TRAJECTORY,  // The cobot is following a trajectory
  } id;


  // The ID of the message that initiated the state.
  uint32_t msg_id;


  // State-specific data.
  union {
    // CALIBRATING
    struct {
      // Bitfield of joints left to calibrate.
      uint8_t joint_bitfield;
    } calibrating;
  };
};
```
