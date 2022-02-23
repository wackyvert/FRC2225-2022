// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  TalonFX shooter = new TalonFX(0);
  VictorSPX feeder = new VictorSPX(13);
  public void end(){
    shooter.set(ControlMode.PercentOutput, 0);
    feeder.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shootBall(){
    shooter.set(ControlMode.PercentOutput, -.9);
  }
  public void feedBall(){
    feeder.set(ControlMode.PercentOutput, -.8);
  }
}
