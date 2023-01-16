package frc.robot.positioning;

import frc.robot.Constants.ArmConstants;

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
