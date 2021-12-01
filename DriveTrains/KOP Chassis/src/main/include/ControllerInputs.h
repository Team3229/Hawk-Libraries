#ifndef CONTROLLER_INPUTS_H
#define CONTROLLER_INPUTS_H

// Method of storing and replaying drivers inputs
struct cmd {
  // Driver 1
  float driver_rightY;
  float driver_rightX;
  float driver_leftX;
};

#endif // CONTROLLER_INPUTS_H