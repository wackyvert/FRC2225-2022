// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShootBall extends CommandBase {
  /** Creates a new ShootBall. */
  public ShootBall(boolean high) {
    addRequirements(RobotContainer.mShooter);
    // Use addRequirements() here to declare subsystem dependencies.
    this.high=high;
  }
  boolean high;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Shooter Spinning up");
    if(high){
    RobotContainer.mShooter.shootBall(10500);
    }
    else
    {RobotContainer.mShooter.shootBall(7600);}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mShooter.end();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
