// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int FrontLeftCAN_ID = 01;
    public static int FrontRightCAN_ID = 03;
    public static int feederID = 13;
    public static int BackLeftCAN_ID = 12;
    public static int BackRightCAN_ID = 02;
    public static int shooterCanID = 0;
    public static int intakeCanID = 8;
    public static int Driver1ID = 0;
    public static final double ksVolts = 0.7246;
    public static final double kvVoltSecondsPerMeter = 1.6275;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16782;
    public static final double kPDriveVel = 2.0844;
    public static final double kTrackwidthMeters = 0.64;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    
}


