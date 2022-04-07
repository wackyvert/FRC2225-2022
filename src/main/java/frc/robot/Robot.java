// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
 //public static Encoder leftEncoder = new Encoder(0, 1);

  SendableChooser<Trajectory> commandSendableChooser;
public Trajectory blueHangar = new Trajectory();
  public Trajectory blueMiddle = new Trajectory();
  public Trajectory blueWall = new Trajectory();
    public Trajectory redHangar = new Trajectory();
    public Trajectory redMiddle = new Trajectory();
    public Trajectory redWall = new Trajectory();
  public  Trajectory trajectory = new Trajectory();
  Field2d m_field = new Field2d();
  @Override
  public void robotInit() {

    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.mDrivetrain, new ArcadeDrive());

    SmartDashboard.putData(m_field);


    
    
    CameraServer.startAutomaticCapture();
    try {

    Path hangarPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallBlueHangar.wpilib.json");
      Path middlePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallBlueMiddle.wpilib.json");
      Path wallPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallBlueWall.wpilib.json");
      Path redhangarPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallRedHangar.wpilib.json");
      Path redmiddlePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallRedMiddle.wpilib.json");
      Path redwallPath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/2BallRedWall.wpilib.json");

      blueHangar = TrajectoryUtil.fromPathweaverJson(hangarPath);
      blueMiddle = TrajectoryUtil.fromPathweaverJson(middlePath);
      blueWall = TrajectoryUtil.fromPathweaverJson(wallPath);
        redHangar = TrajectoryUtil.fromPathweaverJson(redhangarPath);
        redMiddle = TrajectoryUtil.fromPathweaverJson(redmiddlePath);
       redWall = TrajectoryUtil.fromPathweaverJson(redwallPath);

      trajectory=blueHangar;
   } catch (IOException ex) {
      DriverStation.reportError("Error loading traj", ex.getStackTrace());
   }

   m_robotContainer = new RobotContainer();
   // Push the trajectory to Field2d.

    Logger.configureLoggingAndConfig(m_robotContainer, false);
    commandSendableChooser= new SendableChooser<>();
    SmartDashboard.putData(commandSendableChooser);
    commandSendableChooser.setDefaultOption("Blue Hangar",blueHangar);
      commandSendableChooser.addOption("Blue Middle", blueMiddle);
      commandSendableChooser.addOption("Blue Wall",blueWall);
      commandSendableChooser.addOption("Red Hangar",redHangar);
      commandSendableChooser.addOption("Red Middle", redMiddle);
      commandSendableChooser.addOption("Red Wall", redWall);
  }
   


  

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Logger.updateEntries();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Encoder", leftEncoder.getDistance());
  }
@Config
public void setHeading(double value){
    heading=value;
}
public double heading;
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    trajectory=commandSendableChooser.getSelected();
    // Reset odometry to the starting pose of the trajectory.
   RobotContainer.mDrivetrain.resetOdometry(trajectory.getInitialPose(), trajectory.getInitialPose().getRotation().getDegrees());
   m_autonomousCommand = m_robotContainer.getAutonomousCommand(trajectory);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_field.getObject("traj").setTrajectory(trajectory);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
