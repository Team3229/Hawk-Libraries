package frc.robot.DriveSystem.Inputs;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class Controller {
    
    private ControllerType controllerType;
    private GenericHID controller;
    Map<Object, Object> inputs;

    public Controller(ControllerType type, int id) {
        this.controllerType = type;
        this.controller = new GenericHID(id);
        inputs = new HashMap<>();
    }

    public void update() {
        switch(controllerType) {
            case FlightStick:
                inputs.put(Controls.FlightStick.AxisX, controller.getRawAxis(0));
                inputs.put(Controls.FlightStick.AxisY, controller.getRawAxis(1));
                inputs.put(Controls.FlightStick.AxisZ, controller.getRawAxis(2));
                inputs.put(Controls.FlightStick.Throttle, controller.getRawAxis(3));
                
                inputs.put(Controls.FlightStick.Trigger, controller.getRawButton(0));
                inputs.put(Controls.FlightStick.Button2, controller.getRawButton(1));
                inputs.put(Controls.FlightStick.Button3, controller.getRawButton(2));
                inputs.put(Controls.FlightStick.Button4, controller.getRawButton(3));
                inputs.put(Controls.FlightStick.Button5, controller.getRawButton(4));
                inputs.put(Controls.FlightStick.Button6, controller.getRawButton(5));
                inputs.put(Controls.FlightStick.Button7, controller.getRawButton(6));
                inputs.put(Controls.FlightStick.Button8, controller.getRawButton(7));
                inputs.put(Controls.FlightStick.Button9, controller.getRawButton(8));
                inputs.put(Controls.FlightStick.Button10, controller.getRawButton(9));
                inputs.put(Controls.FlightStick.Button11, controller.getRawButton(10));
                inputs.put(Controls.FlightStick.Button12, controller.getRawButton(11));

                inputs.put(Controls.FlightStick.TriggerToggle, controller.getRawButtonPressed(0));
                inputs.put(Controls.FlightStick.Button2Toggle, controller.getRawButtonPressed(1));
                inputs.put(Controls.FlightStick.Button3Toggle, controller.getRawButtonPressed(2));
                inputs.put(Controls.FlightStick.Button4Toggle, controller.getRawButtonPressed(3));
                inputs.put(Controls.FlightStick.Button5Toggle, controller.getRawButtonPressed(4));
                inputs.put(Controls.FlightStick.Button6Toggle, controller.getRawButtonPressed(5));
                inputs.put(Controls.FlightStick.Button7Toggle, controller.getRawButtonPressed(6));
                inputs.put(Controls.FlightStick.Button8Toggle, controller.getRawButtonPressed(7));
                inputs.put(Controls.FlightStick.Button9Toggle, controller.getRawButtonPressed(8));
                inputs.put(Controls.FlightStick.Button10Toggle, controller.getRawButtonPressed(9));
                inputs.put(Controls.FlightStick.Button11Toggle, controller.getRawButtonPressed(10));
                inputs.put(Controls.FlightStick.Button12Toggle, controller.getRawButtonPressed(11));

                inputs.put(Controls.FlightStick.DPad, controller.getPOV());
                
                break;
            case XboxController:
                inputs.put(Controls.XboxController.AButton, controller.getRawButton(0));
                inputs.put(Controls.XboxController.BButton, controller.getRawButton(1));
                inputs.put(Controls.XboxController.XButton, controller.getRawButton(2));
                inputs.put(Controls.XboxController.YButton, controller.getRawButton(3));
                inputs.put(Controls.XboxController.RightStickButton, controller.getRawButton(9));
                inputs.put(Controls.XboxController.LeftStickButton, controller.getRawButton(8));
                inputs.put(Controls.XboxController.StartButton, controller.getRawButton(7));
                inputs.put(Controls.XboxController.SelectButton, controller.getRawButton(6));
                inputs.put(Controls.XboxController.RightBumper, controller.getRawButton(5));
                inputs.put(Controls.XboxController.LeftBumper, controller.getRawButton(4));

                inputs.put(Controls.XboxController.AButtonToggle, controller.getRawButtonPressed(0));
                inputs.put(Controls.XboxController.BButtonToggle, controller.getRawButtonPressed(1));
                inputs.put(Controls.XboxController.XButtonToggle, controller.getRawButtonPressed(2));
                inputs.put(Controls.XboxController.YButtonToggle, controller.getRawButtonPressed(3));
                inputs.put(Controls.XboxController.SelectButtonToggle, controller.getRawButtonPressed(6));
                inputs.put(Controls.XboxController.StartButtonToggle, controller.getRawButtonPressed(7));
                inputs.put(Controls.XboxController.RightBumperToggle, controller.getRawButtonPressed(5));
                inputs.put(Controls.XboxController.LeftBumperToggle, controller.getRawButtonPressed(4));
                inputs.put(Controls.XboxController.RightStickToggle, controller.getRawButtonPressed(9));
                inputs.put(Controls.XboxController.LeftStickToggle, controller.getRawButtonPressed(8));

                inputs.put(Controls.XboxController.LeftTriggerAxis, controller.getRawAxis(2));
                inputs.put(Controls.XboxController.RightTriggerAxis, controller.getRawAxis(3));
                inputs.put(Controls.XboxController.LeftX, controller.getRawAxis(0));
                inputs.put(Controls.XboxController.LeftY, controller.getRawAxis(1));
                inputs.put(Controls.XboxController.RightX, controller.getRawAxis(4));
                inputs.put(Controls.XboxController.RightY, controller.getRawAxis(5));

                inputs.put(Controls.XboxController.DPad, controller.getPOV());

                break;
            default:
                //YOU SUCK, how the heck did this happen mate. Do better.
                System.out.println("You suck");
                break;
            
        }
    }
    
    public Object get(Object input) {
        return inputs.get(input);
    }

    public void rumble(RumbleType rumbleType, double strength) {
        controller.setRumble(rumbleType, strength);
    }
    
    public static class Controls {
        public enum FlightStick {
          AxisX,
          AxisY,
          AxisZ,
          Throttle,

          Trigger,
          Button2,
          Button3,
          Button4,
          Button5,
          Button6,
          Button7,
          Button8,
          Button9,
          Button10,
          Button11,
          Button12,

          TriggerToggle,
          Button2Toggle,
          Button3Toggle,
          Button4Toggle,
          Button5Toggle,
          Button6Toggle,
          Button7Toggle,
          Button8Toggle,
          Button9Toggle,
          Button10Toggle,
          Button11Toggle,
          Button12Toggle,

          DPad
        }
        public enum XboxController {
          RightY,
          RightX,
          RightStickButton,
          RightStickToggle,
          LeftX,
          LeftY,
          LeftStickButton,
          LeftStickToggle,
          AButton,
          BButton,
          XButton,
          XButtonToggle,
          YButton,
          YButtonToggle,
          RightBumper,
          RightBumperToggle,
          LeftBumper,
          LeftBumperToggle,
          RightTriggerAxis,
          LeftTriggerAxis,
          DPad,
          StartButton,
          StartButtonToggle,
          SelectButton,
          SelectButtonToggle,
          AButtonToggle,
          BButtonToggle,
        }
      }

    public enum ControllerType {
        XboxController,
        FlightStick
    }
}