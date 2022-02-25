// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {
    shooter.setInverted(true);
    feeder.setInverted(true);
  }
  BangBangController rpmController = new BangBangController();
  TalonFX shooter = new TalonFX(Constants.shooterCanID);
  VictorSPX feeder = new VictorSPX(Constants.feederID);
  public void end(){
    shooter.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
  }
  double ShooterRPM;
  @Override
  public void periodic() {
    ShooterRPM=shooter.getSelectedSensorVelocity();
    // This method will be called once per scheduler run
  }
  public void shootBall(int rpm){
    shooter.set(ControlMode.PercentOutput, rpmController.calculate(ShooterRPM, rpm));
    if(ShooterRPM>=rpm-100&&ShooterRPM<=rpm+100){
      feedBall();
    }
  }
    public void feedBall(){
    feeder.set(ControlMode.PercentOutput, .8);
  }
}
