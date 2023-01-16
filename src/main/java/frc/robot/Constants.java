// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.positioning.Pose;

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

  public static class RobotDimensions {
    /** The pose of the robot's center */
    public static final Pose CENTER_POSE = new Pose();
    /** The front/back length of the robot's frame in inches */
    public static final double FRAME_LENGTH = 24.0; // Update this when decided
    /** The left/right width of the robot's frame in inches */
    public static final double FRAME_WIDTH = 24.0; // Update this when decided
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
    /** The pose of the arm's base in robot oriented coordinates */
    public static final Pose BASE_POSE = new Pose();
    /** The distance between the floor and the arm base in inches */
    public static final double BASE_DISTANCE_TO_FLOOR = 0.0; // Update this when decided
    /** Length in inches from the base of the arm to the shoulder joint */
    public static final double BASE_TO_SHOULDER_LENGTH = -1.0; // Update this when decided
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

  public static class Limits {
    public static final double MAX_FRAME_EXTENSION = 48.0;
    /** Maximum extended robot height */
    public static final double MAX_EXTENDED_HEIGHT = 78.0;
  }

	/**
	 * Holds constants for things not related to the robot
	 */
	public static class Game {
    public static class Field {
      public static class Grid {
        public static class ConeNode {
          public static class Top {
            /** Height of the node in feet */
            public static final double HEIGHT = 3.8333;
            /** Horizontal distance from the bumper to the node in feet */
            public static final double DISTANCE = 3.3125;
            /** Height of the reflective tape off the ground in feet */
            public static final double TAPE_HEIGHT = 3.4896;
            /** The distance from the top of the tape to the top of the node */
            public static final double ABOVE_TAPE = 0.0156;
          }

          public static class Middle {
            /** Height of the node in feet */
            public static final double HEIGHT = 2.8333;
            /** Horizontal distance from the bumper to the node in feet */
            public static final double DISTANCE = 1.8958;
            /** Height of the reflective tape off the ground in feet */
            public static final double TAPE_HEIGHT = 1.8438;
            /** The distance from the top of the tape to the top of the node */
            public static final double ABOVE_TAPE = 0.6667;
          }
        }

        public static class CubeNode {
          public static class Top {
            /** Height of the node in feet */
            public static final double HEIGHT = 2.9583;
            /** Horizontal distance from the bumper to the middle of the node in feet */
            public static final double DISTANCE = 3.3125;
          }

          public static class Middle {
            /** Height of the node in feet */
            public static final double HEIGHT = 1.9583;
            /** Horizontal distance from the bumper to the middle of the node in feet */
            public static final double DISTANCE = 1.8958;
          }
        }
      }
    }

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
