// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class PID extends CommandBase {
  PIDController pidController;
  /** Creates a new PID. */
  public PID() {
    
    // Use addRequirements() here to declare subsystem dependencies.
  }
  double target = 5;
  Encoder leftEncoder = new Encoder(0,1);
  Encoder rightEncoder = new Encoder(2,3);
  double whd = 6;
  double cpr = 360;
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftEncoder.reset();
    pidController = new PIDController(0.2, 0, 0);
    leftEncoder.setDistancePerPulse(Math.PI*whd/cpr);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.mDrivetrain.setVoltage(pidController.calculate(leftEncoder.getDistance(), target), -pidController.calculate(leftEncoder.getDistance(), target));
    SmartDashboard.putNumber("Encoder", leftEncoder.getDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (leftEncoder.getDistance()-target <1){
      return true;
    }
    else return false;
  }
}
