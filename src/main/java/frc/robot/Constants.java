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
        public static final int CONTROLLER_NUMBER = 0;
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

        /* 
        * Sensor phases for each motor controller.
        * These are in the order of their CAN IDs, but because of the way arrays work, they are off by one from their actual IDs.
        * For example, in order to get the phase for the FL controller, one would call DRIVE_SENSOR_PHASE[Constants.ID_TALON_FRONT_LEFT + 1].
        */
        public static final boolean[] DRIVE_SENSOR_PHASE = {false, false, false, false};

        /* 
        * Invert values for each motor controller.
        * Off by one, see above.
        */
        public static final boolean[] DRIVE_INVERTED = {false, false, false, false};

        /* 
        * Gains for drive PID.
        * Format is DRIVE_GAINS[kP, kI, kD, kF]
        */
        public static final double[] DRIVE_GAINS = {0.0, 0.0, 0.0, 0.0};

        /*
        * Maximum number of RPMs drive velocity PID will have us drive at.
        * 450 should put us at about 12 feet per second
        */
        public static final double DRIVE_VELOCITY_PID_MAX_SPEED = 450.0;

        public static class PID {
            /*TODO: THESE NEED TO BE TUNED */
            public static final double DRIVE_P = 1;
            public static final double DRIVE_I = 0;
            public static final double DRIVE_D = 0;
            public static final double DRIVE_FF= 0.000245;
        }
    }
    /**
     * General constants for the balance command. Primarily used by {@link frc.robot.commands.Balance}.
     */
    public static class BalanceConstants {
        /* Angle threshholds for when the bot is considered balanced and not */
        public static final double OFF_ANGLE_THRESHOLD = 10; 
        public static final double ON_ANGLE_THRESHOLD  = 5;
    }
}
