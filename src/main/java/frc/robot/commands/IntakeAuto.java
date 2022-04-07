package frc.robot.commands;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.RobotContainer;

public class IntakeAuto extends CommandBase {
public IntakeAuto(){
    addRequirements(RobotContainer.mIntake);
}

    @Override
    public void execute(){
        RobotContainer.mIntake.spinIntake();
        System.out.println("bruh");

    }
    @Override
    public void end(boolean interrupted){
        RobotContainer.mIntake.end();
    }

}
