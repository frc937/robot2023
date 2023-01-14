package frc.robot;

/**
 * Kinematics class contains forward and inverse kinematics functions for the
 * 2023 robot arm. All positions are measured relative to the base of the arm.
 * Don't reuse, the math won't work for any other arm.
 *
 * If you want to implement this for a different design, these links describe how.
 * https://motion.cs.illinois.edu/RoboticSystems/Kinematics.html
 * https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
 */
public final class Kinematics {

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

    return armExtension * cosShoulder + Constants.ArmConstants.BASE_TO_SHOULDER_LENGTH;
  }

}
