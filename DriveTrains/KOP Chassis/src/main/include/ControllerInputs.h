#ifndef CONTROLLER_INPUTS_H
#define CONTROLLER_INPUTS_H

// Method of storing and replaying drivers inputs
struct cmd {
  // Driver 1
  double driver_rightY;
  double driver_rightX;
  double driver_leftX;
  int drive_AButton;
  int drive_BButton;
  int drive_XButton;
  int drive_YButton;
  bool drive_RightBumper;
  bool drive_LeftBumper;
  float drive_RightTriggerAxis;
  float drive_LeftTriggerAxis;
  int drive_POV;
  // Driver 2
  float mani_leftX;
  float mani_rightX;
  float mani_leftY;
  float mani_rightY;
  bool mani_AButton;
  bool mani_BButton;
  bool mani_XButton;
  bool mani_YButton;
  bool mani_RightBumper;
  bool mani_LeftBumper;
  float mani_RightTriggerAxis;
  float mani_LeftTriggerAxis;
  int mani_POV;
};

#endif // CONTROLLER_INPUTS_H