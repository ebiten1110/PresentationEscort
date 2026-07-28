#include "serial_receive.h"

#include "head_control.h"
#include "light_control.h"
#include "command.h"
#include "walking.h"


static String inputBuffer = "";
static String lastCommand = "";


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

  Serial.print("Last Command: ");
  Serial.println(lastCommand);

  Serial.println("Manual Turn: V7");
  Serial.println(
    "Auto Track Turn: V8_MICRO_CENTER_STOP"
  );
  Serial.println(
    "Distance Move: V8_3_MICRO_FORWARD_BACKWARD"
  );

  Serial.println(
    "====================================="
  );
}


void printCommandHelp() {
  Serial.println(
    "===== PresentationEscort Commands ====="
  );

  Serial.println("STAND");
  Serial.println("FORWARD");

  Serial.println(
    "LEFT / RIGHT : manual V7 turn"
  );

  Serial.println(
    "TRACK_LEFT / TRACK_RIGHT : "
    "interruptible micro turn"
  );

  Serial.println(
    "TRACK_FORWARD / TRACK_BACKWARD : "
    "distance movement pulse"
  );

  Serial.println("STOP");
  Serial.println("STATUS");
  Serial.println("HELP");

  Serial.println("HEAD_LEFT");
  Serial.println("HEAD_RIGHT");
  Serial.println("HEAD_UP");
  Serial.println("HEAD_DOWN");
  Serial.println("HEAD_CENTER");

  Serial.println("LIGHT_ON");
  Serial.println("LIGHT_OFF");
  Serial.println("LIGHT_TOGGLE");

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

  Serial.print(
    "[SerialReceive] Received="
  );
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
  else if (command == "R") {
    command = CMD_HEAD_UP;
  }
  else if (command == "F") {
    command = CMD_HEAD_DOWN;
  }
  else if (command == "H") {
    command = CMD_HEAD_CENTER;
  }
  else if (command == "L") {
    command = CMD_LIGHT_TOGGLE;
  }

  if (command == CMD_STAND) {
    stand();
  }
  else if (command == CMD_FORWARD) {
    walkForwardOnce();
  }
  else if (command == CMD_LEFT) {
    turnLeft();
  }
  else if (command == CMD_RIGHT) {
    turnRight();
  }
  else if (command == CMD_TRACK_LEFT) {
    trackLeft();
  }
  else if (command == CMD_TRACK_RIGHT) {
    trackRight();
  }
  else if (command == CMD_TRACK_FORWARD) {
    trackForward();
  }
  else if (command == CMD_TRACK_BACKWARD) {
    trackBackward();
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


void initSerialReceive() {
  inputBuffer = "";
  lastCommand = "";

  Serial.println(
    "[SerialReceive] Initialized."
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