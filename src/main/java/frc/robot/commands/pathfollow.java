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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    addRequirements(mDrivetrain);
  }
Trajectory trajectory;
RamseteCommand ramseteCommand;
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



    ramseteCommand =
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


  }
  @Override
  public void execute(){
    ramseteCommand.schedule();
  }

  public void end (boolean interrupted){
    mDrivetrain.setVoltage(0,0);
  }






}
