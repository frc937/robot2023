// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
/* TODO: tune EVERY value in here */
public final class Constants {
  public static class OperatorConstants {
    public static final int CONTROLLER_NUMBER = 0;
  }

  /** General constants for the drivetrain. Primarily used by {@link frc.robot.subsystems.Drive}. */
  public static class Drive {
    /* CAN IDs for the drivetrain motor controllers */
    public static final int ID_TALON_FRONT_LEFT = 0;
    public static final int ID_TALON_FRONT_RIGHT = 1;
    public static final int ID_TALON_REAR_LEFT = 2;
    public static final int ID_TALON_REAR_RIGHT = 3;

    /* Locations of mecanum drive wheels relative to the center of the robot
     * For use with MecanumDriveKinematics
     */
    /* these are real/tuned */
    /* These can go away now because no more mecanum
     * All my hard work, gone :(
     */
    /*
    public static final Translation2d LOCATION_WHEEL_FRONT_LEFT =
        new Translation2d(Units.feetToMeters(10 + (7 / 8)), Units.feetToMeters(8 + (5 / 8)));
    public static final Translation2d LOCATION_WHEEL_FRONT_RIGHT =
        new Translation2d(Units.feetToMeters(10 + (7 / 8)), Units.feetToMeters(8 + (5 / 8)));
    public static final Translation2d LOCATION_WHEEL_REAR_LEFT =
        new Translation2d(Units.feetToMeters(10 + (7 / 8)), Units.feetToMeters(8 + (5 / 8)));
    public static final Translation2d LOCATION_WHEEL_REAR_RIGHT =
        new Translation2d(Units.feetToMeters(10 + (7 / 8)), Units.feetToMeters(8 + (5 / 8)));
     */

    public static final double TRACK_WIDTH = Units.feetToMeters(20.5);

    public static final double DRIVE_ENCODER_PPR = 4096; // CTRE mag encoders

    public static final double WHEEL_SIZE_INCHES = 8; // I THINK they're 8"

    /* pain */
    /* TODO: sysid characterization */
    public static class HolonomicController {
      public static class XController {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
      }

      public static class YController {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
      }

      public static class ThetaController {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static class Constraints {
          public static final double MAX_VELOCITY = 5.0;
          public static final double MAX_ACCELERATION = 3.0;
        }
      }
    }
  }

  /** IDs for the controller buttons. */
  public static class ContollerButtons {
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

  /** General constants for the drivetrain. Primarily used by {@link frc.robot.subsystems.Drive}. */
  public static class DriveConstants {
    /* CAN IDs for the drivetrain motor controllers */
    public static final int ID_TALON_FRONT_LEFT = 0;
    public static final int ID_TALON_FRONT_RIGHT = 1;
    public static final int ID_TALON_REAR_LEFT = 2;
    public static final int ID_TALON_REAR_RIGHT = 3;
  }

  /**
   * General constants for the AStar Subsystem. Primarily used by {@link
   * frc.robot.positioning.AStar}.
   */
  public static class AStar {
    /** Half the length of the field. Positive for Max, Negative for Min. Measured in cm. */
    public static final int FIELD_X = 1654 / 2;

    /** Half the width of the field. Positive for Max, Negative for Min. Measured in cm. */
    public static final int FIELD_Y = 802 / 2;
  }

  /**
   * General constants for the balance command. Primarily used by {@link
   * frc.robot.commands.Balance}.
   */
  public static class BalanceConstants {
    /* Angle threshholds for when the bot is considered balanced and not */
    public static final double OFF_ANGLE_THRESHOLD = 10;
    public static final double ON_ANGLE_THRESHOLD = 5;
  }
}
