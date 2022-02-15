// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain mDrivetrain = new Drivetrain();

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick controller1 = new Joystick(Constants.Driver1ID);

    final JoystickButton aButton1 = new JoystickButton(controller1, 1);
    final JoystickButton bButton1 = new JoystickButton(controller1, 2);
    final JoystickButton xButton1 = new JoystickButton(controller1, 3);
    final JoystickButton yButton1 = new JoystickButton(controller1, 4);
    final JoystickButton rightBumperButton = new JoystickButton(controller1, 5);
    final JoystickButton leftBumperButton = new JoystickButton(controller1, 6);
    final JoystickButton squareButton = new JoystickButton(controller1, 7);
    final JoystickButton startButton = new JoystickButton(controller1, 8);
    final JoystickButton rightJoystickButton = new JoystickButton(controller1, 9);
    final JoystickButton leftJoystickButton = new JoystickButton(controller1, 10);
    aButton1.whenPressed(new PID(), true);
    bButton1.whenPressed(new stopEverything());
    yButton1.whileHeld(new AlignForwardAndSide());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     // Create a voltage constraint to ensure we don't accelerate too fast
     var autoVoltageConstraint =
     new DifferentialDriveVoltageConstraint(
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.ksVoltSecondPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         10);

 // Create config for trajectory
 TrajectoryConfig config =
     new TrajectoryConfig(
             Constants.kMaxSpeedMeters,
             Constants.kMaxAccelMeters)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(Constants.kDriveKinematics)
         // Apply the voltage constraint
         .addConstraint(autoVoltageConstraint);

 // An example trajectory to follow.  All units in meters.
 Trajectory exampleTrajectory =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, 0, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(3, 0, new Rotation2d(0)),
         // Pass config
         config);

 RamseteCommand ramseteCommand =
     new RamseteCommand(
         exampleTrajectory,
         mDrivetrain::getPose,
         new RamseteController(),
         new SimpleMotorFeedforward(
             Constants.ksVolts,
             Constants.ksVoltSecondPerMeter,
             Constants.kaVoltSecondsSquaredPerMeter),
         Constants.kDriveKinematics,
         mDrivetrain::getWheelSpeed,
         new PIDController(.2, 0, 0),
         new PIDController(.2, 0, 0),
         // RamseteCommand passes volts to the callback
         mDrivetrain::setVoltage,
         mDrivetrain);

 // Reset odometry to the starting pose of the trajectory.
 mDrivetrain.resetOdometry(exampleTrajectory.getInitialPose());

 // Run path following command, then stop at the end.
 return ramseteCommand.andThen(() -> mDrivetrain.setVoltage(0, 0));
  }
}

