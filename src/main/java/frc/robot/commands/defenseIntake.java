package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class defenseIntake extends CommandBase {
    @Override
    public void execute(){
        RobotContainer.mIntake.spinIntakeReverse();
    }
    @Override
    public void end(boolean interrupted){
        RobotContainer.mIntake.end();
    }

}
