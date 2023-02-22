// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    
    public static final class DrivebaseConstants {
        public static final double kTrackwidthMeters = 1.7864;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
        public static final double ksVolts = 0.206;
        public static final double kvVoltSecondsPerMeter = 1.282;
        public static final double kaVoltSecondsSquaredPerMeter = 0.237;
        public static final double kPDriveVel = 1.752;

        public static final int LEFT_DRIVE_GROUP_CURRENT_LIMIT = 60;
        public static final int RIGHT_DRIVE_GROUP_CURRENT_LIMIT = 60;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
        public static final double GEARING = 10.0;

      //will make arm constants 

      }
      public static final class RotaryConstants {
        public static final double kp = 0; 
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double kFF = 0;

        public static final double kMinOutput = 0;  
        public static final double kMaxOutput = 0;  

        public static final double kRampRate = 0;
        
        public static final double ConversionFactor = 1;
        public static final int kCurrentLimit = 0;
        public static final double kGearing = 0;
        public static final float kForwardLimit = 0;
        public static final float kReverseLimit = 0;
        public static final boolean kEnableReverseLimit = false;
        public static final boolean kEnableForwardLimit = false;
      }
    
      public static final class AutoConstants {
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
      }
    
      
}
