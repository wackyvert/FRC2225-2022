// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  TalonSRX frontLeft = new TalonSRX(Constants.FrontLeftCAN_ID);
    TalonSRX frontRight = new TalonSRX (Constants.FrontRightCAN_ID);
    TalonSRX backLeft = new TalonSRX (Constants.BackLeftCAN_ID);
    TalonSRX backRight = new TalonSRX(Constants.BackRightCAN_ID);
    public Drivetrain() {
    }
    public void stop (){
      frontLeft.set(ControlMode.PercentOutput, 0);
      frontRight.set(ControlMode.PercentOutput, 0);
      frontLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
    }
    public void visionVoltage(double speed, double turn){
      frontLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
      frontLeft.set(ControlMode.PercentOutput, speed+turn);
      backLeft.follow(frontLeft);
      frontRight.set(ControlMode.PercentOutput, speed-turn);
      backRight.follow(frontRight);
    }
    public void setVoltage(double left, double right){
      frontLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
      backLeft.follow(frontLeft);
      backRight.follow(frontRight);
      frontLeft.set(ControlMode.PercentOutput, left);
      frontRight.set(ControlMode.PercentOutput, right);
    }
  public void arcadeDrive (){
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    backLeft.follow(frontLeft);
    frontRight.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()-OperatorInput.getTurnVal());
    backRight.follow(frontRight);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
