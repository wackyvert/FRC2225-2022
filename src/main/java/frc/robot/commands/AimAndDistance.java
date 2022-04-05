package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;


import static frc.robot.RobotContainer.mDrivetrain;
import static frc.robot.RobotContainer.mLimelight;

public class AimAndDistance extends CommandBase {
    public AimAndDistance(){
        addRequirements(mDrivetrain, mLimelight);
    }
    double ty, ta, tx;
    double xErr, distErr;
    PIDController xController = new PIDController(.2,0,0);
    PIDController yController = new PIDController(.2,0,0);
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
         updateTracking();
        mDrivetrain.visionVoltage(yController.calculate(distErr,0), xController.calculate(xErr, 0));

    }
    public void updateTracking(){
        xErr=-tx;
        distErr= mLimelight.getDistanceFromLimelightToGoalInches();
        ty = mLimelight.getTy();
        tx = mLimelight.getTx();
        ta = mLimelight.getTa();


    }
}
