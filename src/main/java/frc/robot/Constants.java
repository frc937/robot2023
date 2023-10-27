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
    public static final int CONTROLLER_NUMBER = 0;
    public static final int JOYSTICK_NUMBER = 1;
  }

  public static class RobotDimensions {
    /** The pose of the robot's center */
    public static final Pose CENTER_POSE = new Pose(0, 0, 8.5);

    /** The front/back length of the robot's frame in inches */
    public static final double FRAME_LENGTH = 29.0;

    /** The left/right width of the robot's frame in inches */
    public static final double FRAME_WIDTH = 29.0;
  }

  /** General constants for the drivetrain. Primarily used by {@link frc.robot.subsystems.Drive}. */
  public static class DriveConstants {
    /* CAN IDs for the drivetrain motor controllers */
    public static final int ID_TALON_FRONT_LEFT = 3;
    public static final int ID_TALON_FRONT_RIGHT = 1;
    public static final int ID_TALON_REAR_LEFT = 4;
    public static final int ID_TALON_REAR_RIGHT = 2;
  }

  /* TODO: reorganize this class after Cow Town, make separate classes for each individual
   * sub-subsystem
   */
  public static class Arm {
    /** The pose of the arm's base in robot oriented coordinates */
    public static final Pose BASE_POSE = new Pose(0, 0, 8.5);

    /** The distance between the floor and the arm base in inches */
    public static final double BASE_DISTANCE_TO_FLOOR = 8.5;

    /** Length in inches from the base of the arm to the shoulder joint */
    public static final double BASE_TO_SHOULDER_LENGTH = 19;

    /**
     * How high above the arm base should we keep the end effector to stop the arm from stabbing the
     * robot? This only matters inside the frame perimeter. Distance in inches.
     */
    public static final double KEEP_OUT_HEIGHT = 6;

    public static final int ID_TALON_ARM_SHOULDER = 6;
    public static final int ID_TALON_ARM_WINCH = 2; /* this is actually a Talon, not a Talon SRX */
    public static final int ID_TALON_ARM_CLAW = 0; /* This is actually a Spark */
    public static final int ID_TALON_ARM_BASE = 5;

    public static final int CHANNEL_ANALOG_PRESSURE_SENSOR = 0;

    public static final double SETPOINT_PRESSURE_CONE = 5.7;
    public static final double SETPOINT_PRESSURE_CUBE = 5.5;

    public static final boolean INVERTED_TALON_SENSOR_ARM_SHOULDER = true;
    public static final boolean INVERTED_TALON_SENSOR_ARM_BASE = true;

    public static final boolean INVERTED_TALON_ARM_SHOULDER = false;
    public static final boolean INVERTED_TALON_ARM_BASE = false;

    public static final boolean AUTO_ZERO_REVERSE_LIMIT_SHOULDER = true;
    public static final boolean AUTO_ZERO_REVERSE_LIMIT_BASE = true;

    public static final double MIN_LENGTH_ARM_EXTENDER = 30.5;

    public static final double EXTRA_LENGTH_ARM_EXTENDER = 26;

    public static final double DONE_THRESHOLD_ARM_EXTENSION = 1.0;
    /* DONE_THRESHOLD_ARM_CLAW is not tuned yet - update it when implementing pressure sensor */
    public static final double DONE_THRESHOLD_ARM_CLAW = 0.1;

    public static final double CONE_PRESSURE_THRESHOLD = 4.0;

    public static final double SPEED_WINCH_ARM_EXTENSION = 1.0;

    public static final double SPEED_ARM_CLAW = 1;

    public static final double SPEED_ARM_SHOULDER_HOMING = 0.5;
    public static final double SPEED_ARM_BASE_HOMING = 0.5;

    public static class ShoulderPID {
      public static final double kP = 1;
      public static final double kI = 0.002;
      public static final double kD = 100;
      public static final double kFF = 0.0;
      public static final double kIZone = 800;
      public static final double ACCEPTABLE_ERROR = 50;
    }

    public static class BasePID {
      public static final double kP = 1;
      public static final double kI = 0.001;
      public static final double kD = 75;
      public static final double kFF = 0.0;
      public static final double kIZone = 150;
      public static final double ACCEPTABLE_ERROR = 300;
    }

    public static class Poses {
      public static final Pose RESET = new Pose(0, 0, 44);
      public static final Pose SCORE_LOWER = new Pose(0, 23, -6);
      public static final Pose SCORE_MID_CONE = new Pose(0, 38, 32);
      public static final Pose SCORE_MID_CUBE = new Pose(0, 38, 18);
      public static final Pose SCORE_HIGH_CONE = new Pose(0, 55, 44);
      public static final Pose SCORE_HIGH_CUBE = new Pose(0, 55, 28);
      public static final Pose PICKUP = new Pose(-1, 32, -8);
      public static final Pose HUMAN_SHELF = new Pose(0, 27, 32);
      public static final Pose CARRYING = new Pose(0, 0, 44);
    }
  }

  public static class I2C {
    public static int DISTANCE_SENSOR_MULTIPLEXER_PORT = 0;
    public static int COLOR_SENSOR_MULTIPLEXER_PORT = 1;
  }

  public static class Limits {
    /** Maximum horizontal extension over the frame in inches */
    public static final double MAX_FRAME_EXTENSION = 48.0;

    /** Maximum extended robot height in inches */
    public static final double MAX_EXTENDED_HEIGHT = 78.0;

    /** Minimum distance from overextending we want to keep in inches */
    public static final double OVEREXTENSION_DANGER_DISTANCE = 4.0;
  }

  /** Holds constants for things not related to the robot */
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

          /** The pressure reading required to securely hold the cube */
          public static final double PRESSURE_TO_HOLD = -1;
        }
      }
    }
  }

  /**
   * General constants for the balance command. Primarily used by {@link
   * frc.robot.commands.Balance}.
   */
  public static class BalanceConstants {
    /* Angle threshholds for when the bot is considered balanced and not */
    public static final double OFF_ANGLE_THRESHOLD = 10;
    public static final double ON_ANGLE_THRESHOLD = 5;
    public static final double SPEED_MULTIPLIER = 0.4;
  }

  public static class Plunger {
    public static final int ID_PWMMOTOR_PLUNGER = 1;
    public static final double SPEED_PLUNGER_DEPLOY = 0.2;
  }

  public static class Camera {
    public static final int PORT_CAMERA_AIM = 0;
  }
}
