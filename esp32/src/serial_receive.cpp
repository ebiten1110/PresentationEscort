#include "serial_receive.h"

#include "head_control.h"
#include "light_control.h"
#include "command.h"
#include "walking.h"


static String inputBuffer = "";
static String lastCommand = "";


// =====================================================
// 内部関数
// =====================================================

static String normalizeCommand(
  String command
) {
  command.trim();
  command.toUpperCase();
  return command;
}


static void printStatus() {
  Serial.println(
    "===== PresentationEscort STATUS ====="
  );

  Serial.print("Walking State: ");
  Serial.println(getWalkingStateName());

  Serial.print("Busy: ");
  Serial.println(
    isWalkingBusy() ? "YES" : "NO"
  );

  Serial.print("Light: ");
  Serial.println(
    isLightOn() ? "ON" : "OFF"
  );

  Serial.print("Head Current Yaw: ");
  Serial.println(getCurrentHeadYaw(), 1);

  Serial.print("Head Target Yaw: ");
  Serial.println(getTargetHeadYaw(), 1);

  Serial.print("Head Current Pitch: ");
  Serial.println(getCurrentHeadPitch(), 1);

  Serial.print("Head Target Pitch: ");
  Serial.println(getTargetHeadPitch(), 1);

  Serial.print("Head Moving: ");
  Serial.println(
    isHeadMoving() ? "YES" : "NO"
  );

  Serial.print("Last Command: ");
  Serial.println(lastCommand);

  Serial.println(
    "Turn Command Inversion: DISABLED"
  );

  Serial.println(
    "Turn Motion Version: PIVOT_FOOT_V2_STRONG"
  );

  Serial.println(
    "====================================="
  );
}


void printCommandHelp() {
  Serial.println(
    "===== PresentationEscort Commands ====="
  );

  Serial.println("STAND        : stand pose");
  Serial.println("FORWARD      : walk forward once");
  Serial.println("LEFT         : left pivot turn");
  Serial.println("RIGHT        : right pivot turn");
  Serial.println("STOP         : stop walking");
  Serial.println("STATUS       : show robot status");
  Serial.println("HELP         : show command list");

  Serial.println("");
  Serial.println("Short commands:");
  Serial.println("W : FORWARD");
  Serial.println("A : LEFT");
  Serial.println("D : RIGHT");
  Serial.println("S : STOP");
  Serial.println("C : STAND");
  Serial.println("Q : HEAD_LEFT");
  Serial.println("E : HEAD_RIGHT");
  Serial.println("R : HEAD_UP");
  Serial.println("F : HEAD_DOWN");
  Serial.println("H : HEAD_CENTER");
  Serial.println("L : LIGHT_TOGGLE");

  Serial.println(
    "======================================="
  );
}


static void executeCommand(
  String command
) {
  command = normalizeCommand(command);

  if (command.length() == 0) {
    return;
  }

  lastCommand = command;

  Serial.print("[SerialReceive] Received=");
  Serial.println(command);

  if (command == "W") {
    command = CMD_FORWARD;
  }
  else if (command == "A") {
    command = CMD_LEFT;
  }
  else if (command == "D") {
    command = CMD_RIGHT;
  }
  else if (command == "S") {
    command = CMD_STOP;
  }
  else if (command == "C") {
    command = CMD_STAND;
  }
  else if (command == "Q") {
    command = CMD_HEAD_LEFT;
  }
  else if (command == "E") {
    command = CMD_HEAD_RIGHT;
  }
  else if (command == "H") {
    command = CMD_HEAD_CENTER;
  }
  else if (command == "R") {
    command = CMD_HEAD_UP;
  }
  else if (command == "F") {
    command = CMD_HEAD_DOWN;
  }
  else if (command == "L") {
    command = CMD_LIGHT_TOGGLE;
  }

  // 左右の命令入れ替えは行わない。
  if (command == CMD_STAND) {
    stand();
  }
  else if (command == CMD_FORWARD) {
    walkForwardOnce();
  }
  else if (command == CMD_LEFT) {
    Serial.println(
      "[SerialReceive] Dispatch=turnLeft()"
    );
    turnLeft();
  }
  else if (command == CMD_RIGHT) {
    Serial.println(
      "[SerialReceive] Dispatch=turnRight()"
    );
    turnRight();
  }
  else if (command == CMD_STOP) {
    stopWalking();
  }
  else if (command == CMD_STATUS) {
    printStatus();
  }
  else if (command == CMD_HELP) {
    printCommandHelp();
  }
  else if (command == CMD_LIGHT_ON) {
    lightOn();
  }
  else if (command == CMD_LIGHT_OFF) {
    lightOff();
  }
  else if (command == CMD_LIGHT_TOGGLE) {
    lightToggle();
  }
  else if (
    command == CMD_FOLLOW
    || command == CMD_FIX
    || command == CMD_MANUAL
  ) {
    Serial.println(
      "[SerialReceive] "
      "Mode commands are not implemented yet."
    );
  }
  else if (command == CMD_HEAD_LEFT) {
    headLeft();
  }
  else if (command == CMD_HEAD_RIGHT) {
    headRight();
  }
  else if (command == CMD_HEAD_CENTER) {
    headCenter();
  }
  else if (command == CMD_HEAD_UP) {
    headUp();
  }
  else if (command == CMD_HEAD_DOWN) {
    headDown();
  }
  else {
    Serial.print(
      "[SerialReceive] Unknown command: "
    );
    Serial.println(command);
  }
}


// =====================================================
// 公開関数
// =====================================================

void initSerialReceive() {
  inputBuffer = "";
  lastCommand = "";

  Serial.println("[SerialReceive] Initialized.");
  Serial.println(
    "[SerialReceive] "
    "Turn command inversion: DISABLED"
  );
  Serial.println(
    "[SerialReceive] "
    "Turn motion version: PIVOT_FOOT_V2_STRONG"
  );

  printCommandHelp();
}


void updateSerialReceive() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (
      c == '\n'
      || c == '\r'
    ) {
      if (inputBuffer.length() > 0) {
        executeCommand(inputBuffer);
        inputBuffer = "";
      }
    }
    else {
      inputBuffer += c;

      if (inputBuffer.length() > 64) {
        Serial.println(
          "[SerialReceive] "
          "Input too long. Cleared."
        );
        inputBuffer = "";
      }
    }
  }
}


String getLastCommand() {
  return lastCommand;
}