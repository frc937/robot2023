package frc.robot.positioning;

import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
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
  public static boolean isAlmostOverextended(final Pose armPose) {
    final double horizontalExt = getFrameExtension(armPose);
    final double verticalExt = getExtendedRobotHeight(armPose);

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
   * @param armPose the pose of the end effector
   */
  public static boolean isOverextended(final Pose armPose) {
    final double horizontal = getFrameExtension(armPose);
    final double vertical = getExtendedRobotHeight(armPose);

    final double horizontalMax = Constants.Limits.MAX_FRAME_EXTENSION;
    final double verticalMax = Constants.Limits.MAX_EXTENDED_HEIGHT;

    return horizontal > horizontalMax || vertical > verticalMax;
  }

  /**
   * Returns whether the robot is stabbing itself. Keep in mind, this isn't 100%
   * accurate, there could be a last minute fix sticking up. How will you handle
   * that? That sounds like a you problem, not a me problem.
   * Why are you hitting yourself? Why are you hitting yourself?
   * @param armPose the end effector pose being checked
   */
  public static boolean isStabbingSelf(final Pose armPose) {
    final Optional<Pose> frameIntersection = getFramePlaneInstersection(armPose);

    if(frameIntersection.isPresent()) {
      // Outside the frame
      return frameIntersection.get().getZ() < Constants.Arm.KEEP_OUT_HEIGHT;
    } else {
      // Inside the frame
      return armPose.getZ() < Constants.Arm.KEEP_OUT_HEIGHT;
    }
  }

  /**
   * @param x right/left offset from the arm base
   * @param y front/back offset from the arm base
   * @param z up/down offset from the arm base
   * @return arm base rotation
   */
  public static double getArmExtension(final double x, final double y, final double z) {
    final double h = Arm.BASE_TO_SHOULDER_LENGTH;

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
   * @param armPose the end effector pose being checked
   *
   */
  public static double getExtendedRobotHeight(final Pose armPose) {
    final double height = armPose.getWorldOriented(Constants.Arm.BASE_POSE).getZ();
    return height + Constants.Arm.BASE_DISTANCE_TO_FLOOR;
  }

  /**
   * Calculates the height of the robot in inches for the purpose of not
   * overextending. This math only works with vaguely rectangular robots.
   * @param armPose the end effector pose being checked
   */
  public static double getFrameExtension(final Pose armPose) {
    final Pose robotPose = armPose.getWorldOriented(Arm.BASE_POSE);
    final Pose centerPose = robotPose.getWorldOriented(RobotDimensions.CENTER_POSE);
    final Optional<Pose> frameIntersection = getFramePlaneInstersection(armPose);

    if(frameIntersection.isPresent()) {
      // Outside the frame
      final double xDiff = centerPose.getX() - frameIntersection.get().getX();
      final double yDiff = centerPose.getY() - frameIntersection.get().getY();

      return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
    } else {
      // Inside the frame
      return 0.0;
    }
  }

  /**
   * Finds the point at which the arm intersects the plane of the frame
   * @param armPose the pose of the end effector
   */
  public static Optional<Pose> getFramePlaneInstersection(final Pose armPose) {
    // Get end effector pose with respect to the robot center
    final Pose robotPose = armPose.getWorldOriented(Arm.BASE_POSE);
    final Pose centerPose = robotPose.getWorldOriented(RobotDimensions.CENTER_POSE);

    final double x = centerPose.getX();
    final double y = centerPose.getY();
    final double z = centerPose.getZ();
    final double xFrame = RobotDimensions.FRAME_LENGTH / 2;
    final double yFrame = RobotDimensions.FRAME_WIDTH / 2;

    // Skip the math if the frame isn't extended
    if(x < xFrame && x > -xFrame && y < yFrame && y > -yFrame) {
      return Optional.empty();
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

    final double zIntercept = xFrame * z / x;

    return Optional.of(new Pose(xIntercept, yIntercept, zIntercept));
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
	  final double z = armExtension * cosShoulder + Arm.BASE_TO_SHOULDER_LENGTH;

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
    final double heightDiff = z - Arm.BASE_TO_SHOULDER_LENGTH;

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
    return armExtension * cosShoulder + Arm.BASE_TO_SHOULDER_LENGTH;
  }
}
