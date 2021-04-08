package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Controller
{
    private XboxController xboxController;

    public Controller() {
        xboxController = new XboxController(kControllerPort);
    }

    public double getLeftStickX() {  return xboxController.getRawAxis( 0 );  }
    public double getLeftStickY() {  return xboxController.getRawAxis( 1 );  }
    public double getRightStickX() {  return xboxController.getRawAxis( 4 );  }
    public double getRightStickY() {  return xboxController.getRawAxis( 5 );  }

    public double getLeftStickX( boolean applyDeadZone ) {  double val = xboxController.getRawAxis( 0 );  return applyDeadZone ? applyDeadZone(val) : val;  }
    public double getLeftStickY( boolean applyDeadZone ) {  double val = xboxController.getRawAxis( 1 );  return applyDeadZone ? applyDeadZone(val) : val;  }
    public double getRightStickX( boolean applyDeadZone ) {  double val = xboxController.getRawAxis( 4 );  return applyDeadZone ? applyDeadZone(val) : val;  }
    public double getRightStickY( boolean applyDeadZone ) {  double val = xboxController.getRawAxis( 5 );  return applyDeadZone ? applyDeadZone(val) : val;  }

    public boolean getAButton() {  return xboxController.getAButton();  }
    public boolean getBButton() {  return xboxController.getBButton();  }
    public boolean getXButton() {  return xboxController.getXButton();  }
    public boolean getYButton() {  return xboxController.getYButton();  }

    public boolean getDPadUp() {  return xboxController.getPOV() == 0;  }
    public boolean getDPadRight() {  return xboxController.getPOV() == 90;  }
    public boolean getDPadDown() {  return xboxController.getPOV() == 180;  }
    public boolean getDPadLeft() {  return xboxController.getPOV() == 270;  }

    public boolean getLeftBumper() {  return xboxController.getBumper( Hand.kLeft );  }
    public boolean getRightBumper() {  return xboxController.getBumper( Hand.kRight );  }

    public double getLeftTrigger() {  return xboxController.getTriggerAxis( Hand.kLeft ); }
    public double getRightTrigger() {  return xboxController.getTriggerAxis( Hand.kRight ); }

    public double getLeftTrigger( boolean applyDeadZone ) {  double val = xboxController.getTriggerAxis( Hand.kLeft );  return applyDeadZone ? applyDeadZone(val) : val;  }
    public double getRightTrigger( boolean applyDeadZone ) {  double val = xboxController.getTriggerAxis( Hand.kRight );  return applyDeadZone ? applyDeadZone(val) : val;  }

    public boolean getStartButton() {   return xboxController.getStartButton();  }
    public boolean getBackButton() {    return xboxController.getBackButton();  }

    private static double applyDeadZone(double in) {
        if (Math.abs( in ) < kDeadzoneVal)
            return 0.0;
        else
            return in;
    }
}