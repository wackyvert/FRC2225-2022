// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonSRX frontLeft = new TalonSRX(Constants.FrontLeftCAN_ID);
    TalonSRX frontRight = new TalonSRX (Constants.FrontRightCAN_ID);
    TalonSRX backLeft = new TalonSRX (Constants.BackLeftCAN_ID);
    TalonSRX backRight = new TalonSRX(Constants.BackRightCAN_ID);
    Encoder leftEncoder = new Encoder(0,1);
    Encoder rightEncoder = new Encoder(2,3);
  double whd = .1524;
    double cpr = 360;
    private final ADXRS450_Gyro
     m_gyro = new ADXRS450_Gyro();
    public Drivetrain() {
      leftEncoder.setDistancePerPulse(Math.PI*whd/cpr);
      rightEncoder.setDistancePerPulse(Math.PI*whd/cpr);
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }
    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }
    private final DifferentialDriveOdometry m_odometry;
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
    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }
    public void resetEncoders() {
      leftEncoder.reset();
      rightEncoder.reset();
    }
    public void setVoltage(double left, double right){
      left=left/12;
      right=right/12;
      frontLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
     backLeft.set(ControlMode.PercentOutput, left);
     backRight.set(ControlMode.PercentOutput, right);
     frontLeft.set(ControlMode.PercentOutput, left);
      frontRight.set(ControlMode.PercentOutput, right);
    }
  public void arcadeDrive (){
    
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    backLeft.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    frontRight.set(ControlMode.PercentOutput, -OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    backRight.set(ControlMode.PercentOutput, -OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro", m_gyro.getAngle());
    
    
    m_odometry.update(
      m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
}
    // This method will be called once per scheduler run
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
