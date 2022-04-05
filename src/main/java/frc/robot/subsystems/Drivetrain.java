// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.FrontLeftCAN_ID);
  WPI_TalonSRX frontRight = new WPI_TalonSRX (Constants.FrontRightCAN_ID);
  WPI_TalonSRX backLeft = new WPI_TalonSRX (Constants.BackLeftCAN_ID);
  WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.BackRightCAN_ID);
    Encoder leftEncoder = new Encoder(0,1);
    EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
    Encoder rightEncoder = new Encoder(2,3);
    EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
    private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
  KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
  KitbotGearing.k10p71,        // 10.71:1
  KitbotWheelSize.SixInch,     // 6" diameter wheels.
  null                         // No measurement noise.
);
  double whd = .1524;
    double cpr = 360;
    private final ADXRS450_Gyro
     m_gyro = new ADXRS450_Gyro();
     private ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(m_gyro);
     private final Field2d m_field = new Field2d();
    public Drivetrain() {
      SmartDashboard.putData("Field", m_field);
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
    @Log
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
    public void resetOdometry(Pose2d pose, double heading) {
      resetEncoders();
      m_odometry.resetPosition(pose, Rotation2d.fromDegrees(heading));
    }
    public void resetEncoders() {
      leftEncoder.reset();
      rightEncoder.reset();
      
     

    }
    public void setVoltage(double left, double right){
     backLeft.set(ControlMode.PercentOutput, left/12);
     backRight.set(ControlMode.PercentOutput, right/12);
     frontLeft.set(ControlMode.PercentOutput, left/12);
      frontRight.set(ControlMode.PercentOutput, right/12);
    }
  public void arcadeDrive (){
    
    frontLeft.setNeutralMode(NeutralMode.Coast);
    frontRight.setNeutralMode(NeutralMode.Coast);
    frontLeft.set(ControlMode.PercentOutput, -OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    backLeft.set(ControlMode.PercentOutput, -OperatorInput.getSpeedVal()+OperatorInput.getTurnVal());
    frontRight.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()-OperatorInput.getTurnVal());
    backRight.set(ControlMode.PercentOutput, OperatorInput.getSpeedVal()-OperatorInput.getTurnVal());
  }
  @Override
  public void periodic() {
    m_odometry.update(
            m_gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
      m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("gyro", m_gyro.getAngle());
    
    

}
    // This method will be called once per scheduler run
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_driveSim.setInputs(frontLeft.getMotorOutputVoltage()* RobotController.getInputVoltage(), frontRight.getMotorOutputVoltage()*RobotController.getInputVoltage());
    m_driveSim.update(0.02);
    leftEncoderSim .setDistance(m_driveSim.getLeftPositionMeters());
    rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(m_driveSim.getHeading().getDegrees());

    
  }
}
