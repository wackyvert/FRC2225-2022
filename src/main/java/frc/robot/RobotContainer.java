// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.LimelightSubsystem;
import io.github.oblarg.oblog.Logger;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  public static final Shooter mShooter = new Shooter();
  public static final Intake mIntake = new Intake();
  public static final LimelightSubsystem mLimelight = new LimelightSubsystem();



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
    aButton1.whileHeld(new ShootBall(true), true);
    bButton1.whenPressed(new stopEverything());
    yButton1.whileHeld(new AimAndDistance(), true);
    xButton1.whileHeld(new frc.robot.commands.Intake());
    rightBumperButton.whileHeld(new feed());



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand(Trajectory trajectory) {
  /*  var autoVoltageConstraint =
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
    return ramseteCommand.andThen(() -> mDrivetrain.setVoltage(0, 0));*/
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
    //return new SequentialCommandGroup(new ParallelRaceGroup(ramseteCommand, new IntakeAuto()), new stopEverything(), new AimAndDistance(), new ShootBall(true));
return new ShootBall(true);
  }
}


