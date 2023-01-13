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
    public static class ContollerButtons{
        public static final int A_NUMBER = 1;
        public static final int B_NUMBER = 2;
        public static final int X_NUMBER = 3;
        public static final int Y_NUMBER = 4;
        public static final int LEFT_BUMPER_NUMBER = 5;
        public static final int RIGHT_BUMPER_NUMBER = 6;
        public static final int BACK_NUMBER = 7;
        public static final int START_NUMBER = 8;
        public static final int LEFT_STICK_NUMBER = 9;
        public static final int RIGHT_STICK_NUMBER = 10;
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
