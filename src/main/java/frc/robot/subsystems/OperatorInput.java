// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    TalonSRX frontLeft = new TalonSRX(Constants.FrontLeftCAN_ID);
    TalonSRX frontRight = new TalonSRX (Constants.FrontRightCAN_ID);
    TalonSRX backLeft = new TalonSRX (Constants.BackLeftCAN_ID);
    TalonSRX backRight = new TalonSRX(Constants.BackRightCAN_ID);
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
