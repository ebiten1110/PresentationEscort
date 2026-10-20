#ifndef PRESENTATION_ESCORT_SAFETY_RUNTIME_H
#define PRESENTATION_ESCORT_SAFETY_RUNTIME_H

#include <Arduino.h>

// Sole firmware gateway for serial motion-authorization state.
void initSafetyRuntime();
void updateSafetyRuntime();
bool isSafetyMotionAllowed();
bool handleSafetyProtocolCommand(const String& command);
void printSafetyStatus();
void safetyDisarmAndStop();
#endif
