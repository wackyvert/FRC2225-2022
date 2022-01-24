/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ScaleInputs;

import java.util.ArrayList;

public class OperatorInput extends SubsystemBase {
  
  private static XboxController controller1 = new XboxController(0);
  private static Joystick sidewinder = new Joystick(1);
  /**
   * Creates a new OI.
   */
  public OperatorInput() {

  }
  //gets the speed val to drive the robot at (based on the triggers on the primary controller)
  public static double getSpeedVal() {
    return ScaleInputs.scaleInputs(-controller1.getRawAxis(2) + controller1.getRawAxis((3)));
  }
  //gets the turtn val to drive the robot at (based on the right joystick on the primary controller)
  public static double getTurnVal() {
    return -ScaleInputs.scaleInputs(controller1.getRawAxis(0), 0.2, 0.1, 2);
  }
  public static double calculateLeftSpeed() {
    double left = getTurnVal()-getSpeedVal(); //left is speed minus turn for arcade drive
    return left;
  }
  public static double calculateRightSpeed() { //right is speed plus turn for arcade drive
    double right = getTurnVal()+getSpeedVal();
    return right;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
