#ifndef MOTION_LIBRARY_H
#define MOTION_LIBRARY_H

#include <Arduino.h>

struct MotionStep {
  int wait;
  int ch;
  double angle;
  int duration;
};

struct MotionData {
  const char* name;
  const MotionStep* steps;
  int length;
};

const MotionData& getStandMotion();
const MotionData& getStartWalkMotion();
const MotionData& getForwardWalkMotion();
const MotionData& getStopWalkMotion();

const MotionData& getTurnLeftMotion();
const MotionData& getTurnRightMotion();

const MotionData& getTrackLeftMotion();
const MotionData& getTrackRightMotion();

const MotionData& getTrackForwardMotion();
const MotionData& getTrackBackwardMotion();

#endif