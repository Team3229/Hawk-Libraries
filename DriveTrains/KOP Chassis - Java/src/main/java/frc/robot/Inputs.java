// Author: Nick Shreve (3229 Team Member)

// Controller class

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import java.io.Serializable;

/**
 * Inputs class that handles all controller inputs, works for Xbox controllers
 * only.
 */
public class Inputs {
    /** Deadband for the sticks */
    final double STICK_DEADBAND = 0.125;
    /** Deadband for the triggers */
    final double TRIGGER_DEADBAND = 0.1;

    /** Array of controllers */
    public XboxController[] Controllers = { null, null, null, null, null };
    /** Array of rumbles for each controller, used for vibrating controllers */
    public GenericHID[] Rumbles = { null, null, null, null, null };
    /** The count of controllers connected, DO NOT CHANGE */
    public int controllerCount = 0;
    /** Array of the inputs for each of the connected controllers */
    public ControllerInputs[] controllerInputs = { null, null, null, null, null };

    /**
     * Initializes the controllerCount, Controllers, Rumbles, and ControllerInputs
     * items.
     * 
     * @param count (int) The number of controllers, Max 5; Min 1
     */
    public void setupInputs(int count) {
        controllerCount = count;
        for (int i = 0; i < count; i++) {
            Controllers[i] = new XboxController(i);
            Rumbles[i] = new GenericHID(i);
            controllerInputs[i] = new ControllerInputs();
        }
    }

    /**
     * Updates the controls for each connected controller, saves results to
     * integrated ControllerInputs array.
     */
    public void updateControls() {
        // Gets controls for each assigned controller
        for (var i = 0; i < controllerCount; i++) {
            controllerInputs[i].rightY = ((Math.abs(Controllers[i].getRightY()) < STICK_DEADBAND) ? 0
                    : Controllers[i].getRightY());
            controllerInputs[i].rightX = ((Math.abs(Controllers[i].getRightX()) < STICK_DEADBAND) ? 0
                    : Controllers[i].getRightX());
            controllerInputs[i].leftY = ((Math.abs(Controllers[i].getLeftY()) < STICK_DEADBAND) ? 0
                    : Controllers[i].getLeftY());
            controllerInputs[i].leftX = ((Math.abs(Controllers[i].getLeftX()) < STICK_DEADBAND) ? 0
                    : Controllers[i].getLeftX());
            controllerInputs[i].AButton = Controllers[i].getAButton();
            controllerInputs[i].BButton = Controllers[i].getBButton();
            controllerInputs[i].XButton = Controllers[i].getXButton();
            controllerInputs[i].YButton = Controllers[i].getYButton();
            controllerInputs[i].StartButton = Controllers[i].getStartButton();
            controllerInputs[i].BackButton = Controllers[i].getBackButton();
            controllerInputs[i].RightBumper = Controllers[i].getRightBumper();
            controllerInputs[i].LeftBumper = Controllers[i].getLeftBumper();
            controllerInputs[i].RightTriggerAxis = ((Math.abs(Controllers[i].getRightTriggerAxis()) < TRIGGER_DEADBAND)
                    ? 0
                    : Controllers[i].getRightTriggerAxis());
            controllerInputs[i].LeftTriggerAxis = ((Math.abs(Controllers[i].getLeftTriggerAxis()) < TRIGGER_DEADBAND)
                    ? 0
                    : Controllers[i].getLeftTriggerAxis());
            controllerInputs[i].POV = Controllers[i].getPOV();
        }
    }

    /**
     * Resets the controls in the ControllerInputs array
     */
    public void nullControls() {
        for (var i = 0; i < controllerCount; i++) {
            controllerInputs[i].rightY = 0;
            controllerInputs[i].rightX = 0;
            controllerInputs[i].leftY = 0;
            controllerInputs[i].leftX = 0;
            controllerInputs[i].AButton = false;
            controllerInputs[i].BButton = false;
            controllerInputs[i].XButton = false;
            controllerInputs[i].YButton = false;
            controllerInputs[i].StartButton = false;
            controllerInputs[i].BackButton = false;
            controllerInputs[i].RightBumper = false;
            controllerInputs[i].LeftBumper = false;
            controllerInputs[i].RightTriggerAxis = 0;
            controllerInputs[i].LeftTriggerAxis = 0;
            controllerInputs[i].POV = -1;
        }
    }

    /**
     * Controller Inputs class, implemented in Inputs.
     * Do not touch.
     * @see Inputs
     */
    public class ControllerInputs implements Serializable {
        public double rightY;
        public double rightX;
        public double leftX;
        public double leftY;
        public boolean AButton;
        public boolean BButton;
        public boolean XButton;
        public boolean YButton;
        public boolean RightBumper;
        public boolean LeftBumper;
        public double RightTriggerAxis;
        public double LeftTriggerAxis;
        public int POV;
        public boolean StartButton;
        public boolean BackButton;
    }
}