// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.RobotContainer.mDrivetrain;

public class pathfollow extends CommandBase {
  /** Creates a new pathfollow. */
  public pathfollow(Trajectory trajectory) {
    this.trajectory=trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
  }
Trajectory trajectory;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    5);

// Create config for trajectory
    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.


    RamseteCommand ramseteCommand =
            new RamseteCommand(
                    trajectory,
                    mDrivetrain::getPose,
                    new RamseteController(),
                    new SimpleMotorFeedforward(
                            Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    mDrivetrain::getWheelSpeeds,
                    new PIDController(Constants.kPDriveVel, 0, 0),
                    new PIDController(Constants.kPDriveVel, 0, 0),
                    mDrivetrain::setVoltage,
                    mDrivetrain);
    trajectory.getStates();
    ramseteCommand.andThen(() -> mDrivetrain.setVoltage(0, 0));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      // Create a voltage constraint to ensure we don't accelerate too fast
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
