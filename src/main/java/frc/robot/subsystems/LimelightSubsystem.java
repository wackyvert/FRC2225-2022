package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getEntry;

public class LimelightSubsystem extends SubsystemBase {
    double ty;
    double tx;
    double ta;
    double tv;
    double distanceFromLimelightToGoalInches;
    @Override
    public void periodic(){
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        this.tv=tv;
        this.tx=tx;
        this.ty=ty;
        this.ta=ta;
        double targetOffsetAngle_Vertical = getTy();

// how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

// distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

// distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

//calculate distance
        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    }
    public double getDistanceFromLimelightToGoalInches(){
        return distanceFromLimelightToGoalInches;
    }
    public double getTy(){
        return ty;
    }
    public double getTa(){
        return ta;
    }
    public double getTx(){
        return tx;
    }


}
