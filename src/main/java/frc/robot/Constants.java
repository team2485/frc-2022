// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String kRobotIdFile = "/home/lvuser/id.txt";

    public static final class OIConstants {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;
    }

    public static final class FlywheelConstants {
        public static final int kLeftTalonPort = 11;
        public static final int kRightTalonPort = 3;

        public static final int kFlywheeEncoderPort1 = 0;
        public static final int kFlywheelEncoderPort2 = 1;

        public static final int kRevEncoderPulsesPerRevolution = 8192;
        public static final int kRevEncoderSamplesToAverage = 5;
        public static final int kFalconPulsesPerRevolution = 2048;
        
        public static final double kFlywheelRotationsPerPulse = 1.0/4*kFalconPulsesPerRevolution;

        public static final double kFlywheelMaxSpeedRotationsPerSecond = 30;;
        public static final double kS = 0.59475;
        public static final double kV = 0.56738;
        public static final double kA = 0.28161;



        public static final double kP = 1;
        public static final double kD = 0;
    }
}
