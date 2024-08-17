### Messaging

#### Header

All messages sent or received by the firmware are framed with a 3-byte header containing a start
byte, the payload length, and a CCITT-8 CRC. This standard 8-bit CRC uses the polynomial ‘0x07’, an
initial value of ‘0x00’, and no final XOR value.

| Byte | Description                |
| ---- | -------------------------- |
| 0    | Start byte (0x24)          |
| 1    | Payload length             |
| 2    | CRC of payload (crc8ccitt) |
| 3... | Payload...                 |

```cpp
static const uint8_t CRC_TABLE[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


uint8_t crc8ccitt(const void* data, size_t size)
{
  uint8_t val = 0;


  uint8_t* pos = (uint8_t*)data;
  uint8_t* end = pos + size;


  while (pos < end) {
    val = CRC_TABLE[val ^ *pos];
    pos++;
  }


  return val;
}
```

#### Requests (Incoming)

After the header, an incoming request always begins with a byte identifying the request type and a
4-byte ID, which must be unique and is used to match responses to requests. Following these 5 bytes,
some data may be attached in a format defined by the request type.

| Byte | Description  |
| ---- | ------------ |
| 0-2  | ...Header    |
| 3    | Request type |
| 4-7  | Request ID   |
| 8... | Data...      |

##### Initialize

This command initializes the AR3. The host computer must send the expected firmware version to
ensure that it matches the firmware running on the AR3. This is a safety feature to prevent outdated
firmware from behaving unexpectedly in response to inputs designed for newer firmware.

The firmware will respond with an **Ack** or an **Error**.

| Byte | Description               |
| ---- | ------------------------- |
| 0-7  | ...Header, Request        |
| 8-11 | Expected firmware version |

##### Calibrate

This command calibrates the AR3's joints. Each joint is moved in order until they hit their limit
switch. A bitfield is sent that determines which joints should be calibrated and which ones should
not. A joint with a `1` in its position is calibrated. The firmware ensures that joints are not
calibrated in an unsafe order, so depending on the bitfield provided and the state of the AR3, this
command may fail.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command. Once
all joint are calibrated, it will send a **Done** message. If interrupted by a different command, it
will send an **Error**.

| Byte | Description                     |
| ---- | ------------------------------- |
| 0-7  | ...Header, Request              |
| 8    | Bitfield of joints to calibrate |

##### Override

This command overrides the AR3's current joint positions. New values are provided for each desired
joint in the form of an angle in degrees multiplied by 1000. The firmware ensures that the new
positions are valid. All current motion is stopped when this command is run.

The data associated with this command is variable-sized. Only send the joints you want to override.

Importantly, this does not move joints to the provided positions. Instead, it replaces the AR3's
current idea of where its joints are.

The firmware will respond with an **Ack** or an **Error**.

| Byte     | Description                      |
| -------- | -------------------------------- |
| 0-7      | ...Header, Request               |
| N + 8    | Joint ID                         |
| N + 9-12 | New angle (int32) (deg \* 10^-3) |

##### Get Joints

This command requests the AR3's current joint positions. The firmware responds with a **Joints**
message. No data is sent with this command.

##### Move To

This command moves the AR3 to a new set of joint positions. The data associated with this command is
variable-sized. Only send the joints you want to move to.

If `0` is sent for a joint's desired speed, an appropriate acceleration curve is determined
automatically. This is recommended for most scenarios.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command. Once
the AR3 has reached its target positions, it will send a **Done** message. If interrupted by a
different command, it will send an **Error**.

| Byte      | Description                      |
| --------- | -------------------------------- |
| 0-7       | ...Header, Request               |
| N + 8     | Joint ID                         |
| N + 9-12  | New angle (int32) (deg \* 10^-3) |
| N + 13-16 | Speed (int32) (deg \* 10^-3) / s |

##### Move Speed

This command moves joints at a constant speed, indefinitely. The data associated with this command
is variable-sized. Only send the joints you want to move. If a joint reaches its limit, it will
stop.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command.

| Byte     | Description                      |
| -------- | -------------------------------- |
| 0-7      | ...Header, Request               |
| N + 8    | Joint ID                         |
| N + 9-12 | Speed (int32) (deg \* 10^-3) / s |

##### Follow Trajectory

This command moves the AR3 along a trajectory. It follows the same format as **Move To**, but does
not send **Done** when an individual motion is completed. It will also not send an **Error** if it
is interrupted by another command. These changes were made to limit the amount of unecessary
messages being sent when following trajectories generated by MoveIt2.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command.

| Byte      | Description                      |
| --------- | -------------------------------- |
| 0-7       | ...Header, Request               |
| N + 8     | Joint ID                         |
| N + 9-12  | New angle (int32) (deg \* 10^-3) |
| N + 13-16 | Speed (int32) (deg \* 10^-3) / s |

##### Stop

This command stops the AR3's joints. If the first byte is `0`, the joints will be smoothly stopped,
taking into account the motors' maximum acceleration. If the first byte is `1`, the joints will be
stopped immediately. A bitfield must also be provided to determine which joints should be stopped.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command. A
**Done** message will be sent once all joints have stopped, regardless of the stop method.

| Byte | Description                |
| ---- | -------------------------- |
| 0-7  | ...Header, Request         |
| 8    | Stop immediately?          |
| 9    | Bitfield of joints to stop |

##### Go Home

This command moves the AR3 to its home position. A bitfield is sent that determines which joints
should move to their home positions and which ones should not.

The firmware will respond with an **Ack** or an **Error** upon receiving the initial command. Once
the AR3 has reached its home positions, it will send a **Done** message. If interrupted by a
different command, it will send an **Error**.

| Byte | Description                   |
| ---- | ----------------------------- |
| 0-7  | ...Header, Request            |
| 8    | Bitfield of joints to go home |

##### Reset

This command resets the AR3. All joints are stopped and set to their default positions. The firmware
will respond with an **Ack** or an **Error**. No data is sent with this command.

##### Set Log Level

This command sets the AR3's log level. The firmware will respond with an **Ack** or an **Error**.

See **Log Messages** for a table of log levels.

| Byte | Description                 |
| ---- | --------------------------- |
| 0-7  | ...Header, Request          |
| 8    | Log level (0-3, 4 for none) |

##### Set Encoder Feedback

This command enables/disables encoder feedback for individual joints. A bitfield is sent that
determines which joints should have feedback enabled and which ones should not.

The firmware will respond with an **Ack** or an **Error**.

| Byte | Description                                   |
| ---- | --------------------------------------------- |
| 0-7  | ...Header, Request                            |
| 8    | Bitfield of joints to enable/disable feedback |

##### Set Gripper

This command moves the AR3's gripper to a new position. The gripper servo angle is provided in the
form of an integer between `0` and `180`, where `0` is fully open and `180` is fully closed.

The firmware will respond with an **Ack** or an **Error**.

| Byte | Description                         |
| ---- | ----------------------------------- |
| 0-7  | ...Header, Request                  |
| 8    | Gripper servo angle (uint8) (0-180) |

#### Outgoing

There are two classes of outgoing messages: log messages and responses. The AR3 firmware will never
send any data without first being prompted. The only exceptions to this are logs, which may occur at
any time. The first byte of an outgoing message’s payload defines its class: either ‘0x00’ for a
log, or ‘0x01’ for a response.

##### Log Messages

Log messages are sent by the firmware to provide information about the AR3’s state. They are not
responses to any particular request, and they may occur at any time. All logs have an associated
level that indicates their severity. A human-readable message is also included.

| Log Level | Description |
| --------- | ----------- |
| 0         | Debug       |
| 1         | Info        |
| 2         | Warning     |
| 3         | Error       |
| 4         | None        |

| Byte | Description            |
| ---- | ---------------------- |
| 0-2  | ...Header              |
| 3    | 0x00 (indicates a log) |
| 4    | Log level              |
| 5    | Message length         |
| 6... | Message                |

##### Responses

Responses are sent in response to requests. They contain the same ID as the request they are
responding to.

| Byte | Description                 |
| ---- | --------------------------- |
| 0-2  | ...Header                   |
| 3    | 0x01 (indicates a response) |
| 4    | Response type               |
| 5-8  | Command ID                  |
| 9... | Response data               |

###### Ack Response

Acknowledges that a request was successfully received and processed. For short-lived requests (e.g.
**Init**, **Override**, etc.), this response indicates success. For longer-running requests (e.g.
**Move To**, **Calibrate**, etc.), this response indicates that the request was successfully
initiated, but not necessarily completed. The **Done** response will be sent when the request is
completed.

No additional data is sent with this response.

###### Done Response

Indicates that a long-running request has completed successfully.

No additional data is sent with this response.

###### Error Response

Indicates that an error occurred while handling a request. This may happen in place of either an
**Ack** or a **Done** response. A human-readable error message can be included.

If an active request is interrupted by another request (e.g. **Calibrate** is interrupted by **Move
To**), an **Error** of type `6` will be sent in response to the interrupted request.

| Error Code | Meaning                  |
| ---------- | ------------------------ |
| 0          | Other                    |
| 1          | Malformed request        |
| 2          | Out of range             |
| 3          | Invalid joint            |
| 4          | Not initialized          |
| 5          | Not calibrated           |
| 6          | Cancelled                |
| 7          | Invalid firmware version |

| Byte  | Description          |
| ----- | -------------------- |
| 0-8   | ...Header, Response  |
| 9     | Error code           |
| 10    | Error message length |
| 11... | Error message        |

###### Joints Response

Contains the AR3’s current joint positions and speeds. The number of joints is provided as the first
byte. Each joint’s angle is provided as a 4-byte integer in degrees multiplied by 1000. Each joint’s
speed is provided as a 4-byte integer in degrees per second multiplied by 1000. The gripper servo
angle is provided as a single byte.

| Byte      | Description                              |
| --------- | ---------------------------------------- |
| 0-8       | ...Header, Response                      |
| 9         | Number of joints                         |
| N + 10-13 | Joint N angle (int32) (deg \* 10^-3)     |
| N + 14-17 | Joint N speed (int32) (deg \* 10^-3) / s |
| ...       | ...                                      |
| Last byte | Gripper servo angle (uint8) (0-180)      |
