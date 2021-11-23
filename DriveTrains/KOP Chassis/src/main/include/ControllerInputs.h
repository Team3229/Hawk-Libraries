#ifndef CONTROLLER_INPUTS_H
#define CONTROLLER_INPUTS_H

// Method of storing and replaying drivers inputs
struct cmd {
  // Driver 1
  float drive_rightY;
  float drive_rightX;
  float drive_leftY;
  float drive_leftX;
  bool drive_AButton;
  bool drive_BButton;
  bool drive_XButton;
  bool drive_YButton;
  bool drive_RightBumper;
  bool drive_LeftBumper;
  float drive_RightTriggerAxis;
  float drive_LeftTriggerAxis;
  int drive_POV;
};

#endif // CONTROLLER_INPUTS_H