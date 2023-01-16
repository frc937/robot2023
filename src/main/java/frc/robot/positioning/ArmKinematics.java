package frc.robot.positioning;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.RobotDimensions;

/**
 * Kinematics class contains forward and inverse kinematics functions for the
 * 2023 robot arm. All positions are measured relative to the base of the arm.
 * Don't reuse, the math won't work for any other arm.
 *
 * If you want to implement this for a different design, these links describe how.
 * https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html
 * https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
 */
public final class ArmKinematics {
  /**
   * Returns whether or not the arm is in danger of overextending.
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static boolean isAlmostOverextended(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double horizontalExt = getFrameExtension(baseRotation, shoulderRotation, armExtension);
    final double verticalExt = getExtendedRobotHeight(baseRotation, shoulderRotation, armExtension);

    final double dangerZone = Constants.Limits.OVEREXTENSION_DANGER_DISTANCE;
    final double horizontal = horizontalExt + dangerZone;
    final double vertical = verticalExt + dangerZone;

    final double horizontalMax = Constants.Limits.MAX_FRAME_EXTENSION;
    final double verticalMax = Constants.Limits.MAX_EXTENDED_HEIGHT;

    return horizontal > horizontalMax || vertical > verticalMax;
  }

  /**
   * Returns whether or not the arm is overextended. If this is the case, the
   * robot MUST immediately retract the arm to avoid penalties.
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static boolean isOverextended(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double horizontal = getFrameExtension(baseRotation, shoulderRotation, armExtension);
    final double vertical = getExtendedRobotHeight(baseRotation, shoulderRotation, armExtension);

    final double horizontalMax = Constants.Limits.MAX_FRAME_EXTENSION;
    final double verticalMax = Constants.Limits.MAX_EXTENDED_HEIGHT;

    return horizontal > horizontalMax || vertical > verticalMax;
  }

  /**
   * @param x right/left offset from the arm base
   * @param y front/back offset from the arm base
   * @param z up/down offset from the arm base
   * @return arm base rotation
   */
  public static double getArmExtension(final double x, final double y, final double z) {
    final double h = ArmConstants.BASE_TO_SHOULDER_LENGTH;

    return Math.sqrt(x * x + y * y + z * z + h * h - 2 * h * z);
  }

  /**
   * Gets the arm extension to achieve a given end effector pose based at the
   * bottom of the arm. This doesn't take pose orientation into account.
   * @param target the desired pose
   */
  public static double getArmExtension(final Pose target) {
	  return getArmExtension(target.getX(), target.getY(), target.getZ());
  }

  /**
   * @param x right/left offset from the arm base
   * @param y front/back offset from the arm base
   * @param z up/down offset from the arm base
   * @return arm base rotation
   */
  public static double getBaseRotation(final double x, final double y, final double z) {
    return Math.atan2(x, y);
  }

  /**
   * Gets the arm base rotation to achieve a given end effector pose based at the
   * bottom of the arm. This doesn't take pose orientation into account.
   * @param target the desired pose
   */
  public static double getBaseRotation(final Pose target) {
	  return getBaseRotation(target.getX(), target.getY(), target.getZ());
  }

  /**
   * Calculates the height of the robot in inches for the purpose of not overextending
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static double getExtendedRobotHeight(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final Pose pose = getPose(baseRotation, shoulderRotation, armExtension);
    final double height = pose.getWorldOriented(Constants.ArmConstants.BASE_POSE).getZ();
    return height + Constants.ArmConstants.BASE_DISTANCE_TO_FLOOR;
  }

  /**
   * Calculates the height of the robot in inches for the purpose of not
   * overextending. This math only works with vaguely rectangular robots.
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static double getFrameExtension(final double baseRotation, final double shoulderRotation, final double armExtension) {
    // Get end effector pose with respect to the robot center
    final Pose armPose = getPose(baseRotation, shoulderRotation, armExtension);
    final Pose robotPose = armPose.getWorldOriented(ArmConstants.BASE_POSE);
    final Pose centerPose = robotPose.getWorldOriented(RobotDimensions.CENTER_POSE);

    final double x = centerPose.getX();
    final double y = centerPose.getY();
    final double xFrame = RobotDimensions.FRAME_LENGTH / 2;
    final double yFrame = RobotDimensions.FRAME_WIDTH / 2;

    // Skip the math if the frame isn't extended
    if(x < xFrame && x > -xFrame && y < yFrame && y > -yFrame) {
      return 0.0;
    }

    // Figure out which intersection we care about
    boolean isLeft = false;
    boolean isFront = false;
    boolean isRight = false;
    boolean isBack = false;
    final boolean isForward = centerPose.getX() > 0;

    if(isForward) {
      if(y / x > yFrame / xFrame) {
        isLeft = true;
      } else if(y / x > -yFrame / xFrame) {
        isFront = true;
      } else {
        isRight = true;
      }
    } else {
      if(y / x > yFrame / xFrame) {
        isRight = true;
      } else if(y / x > -yFrame / xFrame) {
        isBack = true;
      } else {
        isLeft = true;
      }
    }

    // Calculate frame plane intersection
    double xIntercept = 0.0;
    double yIntercept = 0.0;

    if(isLeft) {
      yIntercept = yFrame;
      xIntercept = yFrame * x / y;
    }

    if(isFront) {
      xIntercept = xFrame;
      yIntercept = xFrame * y / x;
    }

    if(isRight) {
      yIntercept = -yFrame;
      xIntercept = yFrame * x / y;

    }

    if(isBack) {
      xIntercept = -xFrame;
      yIntercept = xFrame * y / x;
    }

    // Calculate frame extension
    final double xDiff = x - xIntercept;
    final double yDiff = y - yIntercept;

    return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
  }

  /**
   * Gets the orientation of the end effector. Keep in mind that Rotation3d uses
   * radians instead of degrees.
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static Rotation3d getOrientation(final double baseRotation, final double shoulderRotation, final double armExtension) {
	  // Since this is a simple enough arm design and the roll will always be 0,
	  // we can just treat the arm extension as an angle/axis orientation.
	  final double sinBase = Math.sin(Math.toRadians(baseRotation));
    final double cosBase = Math.cos(Math.toRadians(baseRotation));
    final double sinShoulder = Math.sin(Math.toRadians(shoulderRotation));
	  final double cosShoulder = Math.cos(Math.toRadians(shoulderRotation));

    final double x = armExtension * sinBase * sinShoulder;
    final double y = -1 * armExtension * cosBase * sinShoulder;
	  final double shoulderBasedZ = armExtension * cosShoulder;

	  final Vector<N3> vector = VecBuilder.fill(x, y, shoulderBasedZ);
	  return new Rotation3d(vector, 0.0);
  }

  /**
   * Gets the full pose of the end effector
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   */
  public static Pose getPose(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double sinBase = Math.sin(Math.toRadians(baseRotation));
    final double cosBase = Math.cos(Math.toRadians(baseRotation));
    final double sinShoulder = Math.sin(Math.toRadians(shoulderRotation));
    final double cosShoulder = Math.cos(Math.toRadians(shoulderRotation));

	  final double x = armExtension * sinBase * sinShoulder;
    final double y = -1 * armExtension * cosBase * sinShoulder;
	  final double shoulderBasedZ = armExtension * cosShoulder;
	  final double z = armExtension * cosShoulder + ArmConstants.BASE_TO_SHOULDER_LENGTH;

	  final Vector<N3> vector = VecBuilder.fill(x, y, shoulderBasedZ);
	  final Rotation3d orientation = new Rotation3d(vector, 0.0);

	  return new Pose(x, y, z, orientation);
  }

  /**
   * @param x right/left offset from the arm base
   * @param y front/back offset from the arm base
   * @param z up/down offset from the arm base
   * @return arm base rotation
   */
  public static double getShoulderRotation(final double x, final double y, final double z) {
    final double distance = Math.sqrt(x * x + y * y);
    final double heightDiff = z - ArmConstants.BASE_TO_SHOULDER_LENGTH;

    return Math.atan2(distance, heightDiff);
  }

  /**
   * Gets the shoulder rotation to achieve a given end effector pose based at the
   * bottom of the arm. This doesn't take pose orientation into account.
   * @param target the desired pose
   */
  public static double getShoulderRotation(final Pose target) {
	  return getShoulderRotation(target.getX(), target.getY(), target.getZ());
  }

  /**
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   * @return X position relative to the arm base.
   */
  public static double getX(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double sinBase = Math.sin(Math.toRadians(baseRotation));
    final double sinShoulder = Math.sin(Math.toRadians(shoulderRotation));

    return armExtension * sinBase * sinShoulder;
  }

  /**
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   * @return Y position relative to the arm base.
   */
  public static double getY(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double cosBase = Math.cos(Math.toRadians(baseRotation));
    final double sinShoulder = Math.sin(Math.toRadians(shoulderRotation));

    return -1 * armExtension * cosBase * sinShoulder;
  }

  /**
   * @param baseRotation counter clockwise rotation of the arm base zeroed
   * on the Y axis in degrees
   * @param shoulderRotation counter clockwise rotation of the shoulder about
   * the X axis zeroed on the Z axis in degrees
   * @param armExtension distance in inches from the shoulder joint to the
   * end of the grabber arm
   * @return Z(up/down) position relative to the arm base.
   */
  public static double getZ(final double baseRotation, final double shoulderRotation, final double armExtension) {
    final double cosShoulder = Math.cos(Math.toRadians(shoulderRotation));
    return armExtension * cosShoulder + ArmConstants.BASE_TO_SHOULDER_LENGTH;
  }
}
