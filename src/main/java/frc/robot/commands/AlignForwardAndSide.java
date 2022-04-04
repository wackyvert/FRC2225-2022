/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class AlignForwardAndSide extends CommandBase {
  /**
   * Creates a new AlignForwardAndSide.
   */
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  public AlignForwardAndSide() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  public void Update_Limelight_Tracking(){
    // These numbers must be tuned for your Robot!  Be careful!
    final double STEER_K = 0.03;                    // how hard to turn toward the target
    final double DRIVE_K = 1.2;                    // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = .675;        // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.6;                   // Simple speed limit so we don't drive too fast
    final double MAX_STEER = 0.15;

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    if (tv < 1.0)
    {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.3;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering
    double steer_cmd = tx * STEER_K;
    if (steer_cmd > MAX_STEER)
    {
      steer_cmd = MAX_STEER;
    }
    if (steer_cmd < -MAX_STEER)
    {
      steer_cmd = -MAX_STEER;
    }
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired areaPb
    double drive_cmd = (Math.sqrt(DESIRED_TARGET_AREA) - Math.sqrt(ta)) * DRIVE_K;
    SmartDashboard.putNumber("ta", ta);

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE)
    {
      drive_cmd = MAX_DRIVE;
    }
    if (drive_cmd < -MAX_DRIVE)
    {
      drive_cmd = -MAX_DRIVE;
    }
    m_LimelightDriveCommand = -drive_cmd;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Update_Limelight_Tracking();
    SmartDashboard.putNumber("Vision Drive", m_LimelightDriveCommand);
    SmartDashboard.putNumber("Vision Steer", m_LimelightSteerCommand);
    SmartDashboard.putBoolean("Vision Target", m_LimelightHasValidTarget);

    if (m_LimelightHasValidTarget)
    {
      RobotContainer.mDrivetrain.visionVoltage(-m_LimelightDriveCommand,m_LimelightSteerCommand);
    }
    }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return  false;
  }
}
