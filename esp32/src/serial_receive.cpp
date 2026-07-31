#include "serial_receive.h"

#include "head_control.h"
#include "light_control.h"
#include "command.h"
#include "walking.h"

static String inputBuffer = "";
static String lastCommand = "";

static String normalizeCommand(String command) {
  command.trim();
  command.toUpperCase();
  return command;
}

static bool handleHeadSetCommand(const String& command) {
  if (command.startsWith("HEAD_POSE_SET:")) {
    const String payload = command.substring(14);
    const int commaIndex = payload.indexOf(',');

    if (commaIndex <= 0 || commaIndex >= payload.length() - 1) {
      Serial.println("[SerialReceive] Invalid HEAD_POSE_SET format.");
      return true;
    }

    const float yaw = payload.substring(0, commaIndex).toFloat();
    const float pitch = payload.substring(commaIndex + 1).toFloat();
    setHeadPoseTarget(yaw, pitch);
    return true;
  }

  if (command.startsWith("HEAD_YAW_SET:")) {
    const float yaw = command.substring(13).toFloat();
    setHeadYawTarget(yaw);
    return true;
  }

  if (command.startsWith("HEAD_PITCH_SET:")) {
    const float pitch = command.substring(15).toFloat();
    setHeadPitchTarget(pitch);
    return true;
  }

  return false;
}

static void printStatus() {
  Serial.println("===== PresentationEscort STATUS =====");
  Serial.print("Walking State: ");
  Serial.println(getWalkingStateName());
  Serial.print("Busy: ");
  Serial.println(isWalkingBusy() ? "YES" : "NO");
  Serial.print("RGB Light: ");
  Serial.println(getLightModeName());
  Serial.print("Last Command: ");
  Serial.println(lastCommand);
  printHeadStatus();
  Serial.println("Control: V8.8_DUAL_LOOP_CAMERA_BODY");
  Serial.println("=====================================");
}

void printCommandHelp() {
  Serial.println("===== PresentationEscort Commands =====");
  Serial.println("STAND / FORWARD / LEFT / RIGHT / STOP");
  Serial.println("TRACK_LEFT / TRACK_RIGHT");
  Serial.println("TRACK_FORWARD / TRACK_BACKWARD");
  Serial.println("HEAD_LEFT / HEAD_RIGHT / HEAD_UP / HEAD_DOWN");
  Serial.println("HEAD_CENTER / HEAD_STATUS");
  Serial.println("HEAD_POSE_SET:<yaw>,<pitch>");
  Serial.println("HEAD_YAW_SET:<yaw>");
  Serial.println("HEAD_PITCH_SET:<pitch>");
  Serial.println("RGB_OFF / RGB_WHITE / RGB_RED");
  Serial.println("RGB_GREEN / RGB_BLUE / RGB_YELLOW");
  Serial.println("RGB_PURPLE / RGB_ERROR");
  Serial.println("STATUS / HELP");
  Serial.println("=======================================");
}

static void executeCommand(String command) {
  command = normalizeCommand(command);
  if (command.length() == 0) return;

  lastCommand = command;

  // 高頻度の絶対角度・状態要求はログを省略し、
  // シリアル帯域とログ書き込み負荷を抑える。
  if (handleHeadSetCommand(command)) return;
  if (command == CMD_HEAD_STATUS) {
    printHeadStatus();
    return;
  }

  Serial.print("[SerialReceive] Received=");
  Serial.println(command);

  if (command == "W") command = CMD_FORWARD;
  else if (command == "A") command = CMD_LEFT;
  else if (command == "D") command = CMD_RIGHT;
  else if (command == "S") command = CMD_STOP;
  else if (command == "C") command = CMD_STAND;
  else if (command == "Q") command = CMD_HEAD_LEFT;
  else if (command == "E") command = CMD_HEAD_RIGHT;
  else if (command == "R") command = CMD_HEAD_UP;
  else if (command == "F") command = CMD_HEAD_DOWN;
  else if (command == "H") command = CMD_HEAD_CENTER;
  else if (command == "L") command = CMD_LIGHT_TOGGLE;

  if (command == CMD_STAND) stand();
  else if (command == CMD_FORWARD) walkForwardOnce();
  else if (command == CMD_LEFT) turnLeft();
  else if (command == CMD_RIGHT) turnRight();
  else if (command == CMD_TRACK_LEFT) trackLeft();
  else if (command == CMD_TRACK_RIGHT) trackRight();
  else if (command == CMD_TRACK_FORWARD) trackForward();
  else if (command == CMD_TRACK_BACKWARD) trackBackward();
  else if (command == CMD_STOP) stopWalking();
  else if (command == CMD_STATUS) printStatus();
  else if (command == CMD_HELP) printCommandHelp();
  else if (command == CMD_LIGHT_ON) lightOn();
  else if (command == CMD_LIGHT_OFF) lightOff();
  else if (command == CMD_LIGHT_TOGGLE) lightToggle();
  else if (command == CMD_RGB_OFF) lightOff();
  else if (command == CMD_RGB_WHITE) lightWhite();
  else if (command == CMD_RGB_RED) lightRed();
  else if (command == CMD_RGB_GREEN) lightGreen();
  else if (command == CMD_RGB_BLUE) lightBlue();
  else if (command == CMD_RGB_YELLOW) lightYellow();
  else if (command == CMD_RGB_PURPLE) lightPurple();
  else if (command == CMD_RGB_ERROR) lightError();
  else if (command == CMD_HEAD_LEFT) headLeft();
  else if (command == CMD_HEAD_RIGHT) headRight();
  else if (command == CMD_HEAD_CENTER) headCenter();
  else if (command == CMD_HEAD_UP) headUp();
  else if (command == CMD_HEAD_DOWN) headDown();
  else if (
    command == CMD_FOLLOW
    || command == CMD_FIX
    || command == CMD_MANUAL
  ) {
    Serial.println("[SerialReceive] Mode command not implemented.");
  }
  else {
    Serial.print("[SerialReceive] Unknown command: ");
    Serial.println(command);
  }
}

void initSerialReceive() {
  inputBuffer = "";
  lastCommand = "";
  Serial.println("[SerialReceive] Initialized.");
  printCommandHelp();
}

void updateSerialReceive() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        executeCommand(inputBuffer);
        inputBuffer = "";
      }
    }
    else {
      inputBuffer += c;
      if (inputBuffer.length() > 96) {
        Serial.println("[SerialReceive] Input too long. Cleared.");
        inputBuffer = "";
      }
    }
  }
}

String getLastCommand() { return lastCommand; }
