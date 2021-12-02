#ifndef CONTROLLER_INPUTS_H
#define CONTROLLER_INPUTS_H

// Method of storing and replaying drivers inputs
struct cmd {
  // Driver 1
  double driver_rightY;
  double driver_rightX;
  double driver_leftX;
  bool drive_AButton;
  bool drive_BButton;
  bool drive_XButton;
  bool drive_YButton;
};

#endif // CONTROLLER_INPUTS_H