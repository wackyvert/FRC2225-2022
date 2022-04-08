package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class driveback extends CommandBase {
   boolean done;
    @Override
    public void initialize(){
RobotContainer.mDrivetrain.resetEncoders();

    }
    public void execute(){
        RobotContainer.mDrivetrain.setVoltage(-4.5,-4.5);
    }
    @Override
    public void end(boolean interrupted){
        RobotContainer.mDrivetrain.stop();
    }

@Override public boolean isFinished(){
      return false;

}
}
