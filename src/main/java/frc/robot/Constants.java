// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    /**
     * General constants for the drivetrain. Primarily used by {@link frc.robot.subsystems.Drive}.
     */
    public static class DriveConstants {
        /* CAN IDs for the drivetrain motor controllers */
        public static final int ID_TALON_FRONT_LEFT = 0;
        public static final int ID_TALON_FRONT_RIGHT = 1;
        public static final int ID_TALON_REAR_LEFT = 2;
        public static final int ID_TALON_REAR_RIGHT = 3;
    }

    public static class ArmConstants {
        public static final double BASE_TO_SHOULDER_LENGTH = -1.0;
        public static final int ID_TALON_ARM_SHOULDER = 4;
        public static final int ID_TALON_ARM_WINCH = 5;

        public static final boolean INVERTED_TALON_SENSOR_ARM_SHOULDER = false;
        public static final boolean INVERTED_TALON_SENSOR_ARM_EXTENSION = false;

        public static final boolean INVERTED_TALON_ARM_SHOULDER = false;
        public static final boolean INVERTED_TALON_ARM_EXTENSION = false;

        public static class ShoulderPID {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.0;
        }

        public static class ExtensionPID {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.0;

        }
    }
}
