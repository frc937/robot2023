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
        /** Length in inches from the base of the arm to the shoulder joint */
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

	/**
	 * Holds constants for things not related to the robot
	 */
	public static class Game {
		public static class Objects {
			public static class Cone {
				/** Height in inches */
				public static final double HEIGHT = 12.8125;
				/** Width of the square base in inches */
				public static final double WIDTH = 8.375;
				/** Diameter of the bottom of the cone */
				public static final double BOTTOM_DIAMETER = 6.625;
				/** Diameter of top bottom of the cone */
				public static final double TOP_DIAMETER = 1.75;
				/** Average diameter of the cone's grabbable area */
				public static final double AVG_DIAMETER = 4.1875;
				/** The pressure reading required to securely hold the cone */
				public static final double PRESSURE_TO_HOLD = -1;
			}

			public static class Cube {
				/** Side length of a properly inflated cube in inches */
				public static final double LENGTH = 9.5;
				/** The pressure reading required to securely hold the cone */
				public static final double PRESSURE_TO_HOLD = -1;
			}
		}
	}
}
