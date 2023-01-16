package frc.robot.positioning;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Keeps track of a position and orientation with most of the gross math out of
 * the way
 */
public class Pose extends Pose3d {
  /**
   * Constructs a Pose at the origin with default orientation
   */
  public Pose() {
    super();
  }

  /**
   * Constructs a Pose from a Pose3d
   */
  public Pose(final Pose3d rhs) {
    super(rhs.getTranslation(), rhs.getRotation());
  }

  /**
   * Constructs a Pose at x,y,z with default orientation
   */
  public Pose(final double x, final double y, final double z) {
    super(x, y, z, new Rotation3d());
  }

  /**
   * Constructs a Pose at x,y,z with an orientation described by roll,pitch,yaw
   * in degrees
   */
  public Pose(final double x, final double y, final double z, final Rotation3d rotation) {
    super(x, y, z, rotation);
  }

  /**
   * Returns this pose as a wpilib Pose3d
   * This doesn't lose any information because the Pose class holds no additional
   * information.
   */
  public Pose3d getPose3d() {
    return new Pose3d(this.getTranslation(), this.getRotation());
  }

  /**
   * Returns this pose, but in world based coordinates
   * This is very useful if you have a pose where (0,0,0) is on the robot. If
   * you know where that (0,0,0) point is oriented in the world, then this
   * method will give you a new pose with reference to the world instead of the
   * robot.
   * <p>
   * This method can be chained if you want to find the pose of an arm in the
   * world. If the arm pose is based at the bottom of the robot, and the robot
   * can move around in the world, you can chain to find world coordinates.
   * First, you would find convert from arm coordinates to robot coordinates.
   * Then, you would convert robot coordinates to world coordinates.
   * @param origin the world pose describing the origin of this pose
   */
  public Pose getWorldOriented(final Pose origin) {
	  final Pose3d worldOrigin = new Pose3d();
	  final Pose3d poseOrigin = origin.getPose3d();
	  final Transform3d transformation = new Transform3d(poseOrigin, worldOrigin);

	  return new Pose(this.transformBy(transformation));
  }

  /**
   * Returns the counterclockwise rotation angle around the X axis (roll) in degrees
   */
  public double getXRot() {
    return Math.toDegrees(this.getRotation().getX());
  }

  /**
   * Returns the counterclockwise rotation angle around the Y axis (pitch) in degrees
   */
  public double getYRot() {
    return Math.toDegrees(this.getRotation().getY());
  }

  /**
   * Returns the counterclockwise rotation angle around the Z axis (yaw) in degrees
   */
  public double getZRot() {
    return Math.toDegrees(this.getRotation().getZ());
  }

  /**
   * Pitches the current pose up/down
   * 3D rotations can get confusing quick, this will pitch the pose as if it were
   * a plane flying in its starting orientation.
   * @param pitch number of degrees to pitch by
   */
  public void pitch(final double pitch) {
    final Translation3d translation = new Translation3d();
    final Rotation3d startingRotation = this.getRotation();

    // Undo the starting rotation
    this.transformBy(new Transform3d(translation, startingRotation.unaryMinus()));

    // Apply pitch
    final double pitchRad = Math.toRadians(pitch);
    final Rotation3d rotation = new Rotation3d(0, pitchRad, 0);
    this.transformBy(new Transform3d(translation, rotation));

    // Reapply starting rotation
    this.transformBy(new Transform3d(translation, startingRotation));
  }

  /**
   * Rolls the current pose counter clockwise
   * 3D rotations can get confusing quick, this will roll the pose as if it were
   * a plane flying in its starting orientation.
   * @param roll number of degrees to roll by
   */
  public void roll(final double roll) {
    final Translation3d translation = new Translation3d();
    final Rotation3d startingRotation = this.getRotation();

    // Undo the starting rotation
    this.transformBy(new Transform3d(translation, startingRotation.unaryMinus()));

    // Apply roll
    final double rollRad = Math.toRadians(roll);
    final Rotation3d rotation = new Rotation3d(rollRad, 0, 0);
    this.transformBy(new Transform3d(translation, rotation));

    // Reapply starting rotation
    this.transformBy(new Transform3d(translation, startingRotation));
  }

  /**
   * Rotates the pose
   * @param rotation the rotation to apply to this pose
   */
  public void rotate(final Rotation3d rotation) {
    final Translation3d translation = new Translation3d();
    this.transformBy(new Transform3d(translation, rotation));
  }

  /**
   * Rotates the pose around the x, y, and z axes
   * 3D rotations can get confusing quick, this will change the pose in the same
   * way no matter how it's oriented beforehand.
   */
  public void rotate(final double x, final double y, final double z) {
    final double xRad = Math.toRadians(x);
    final double yRad = Math.toRadians(y);
    final double zRad = Math.toRadians(z);

    final Translation3d translation = new Translation3d();
    final Rotation3d rotation = new Rotation3d(xRad, yRad, zRad);

    this.transformBy(new Transform3d(translation, rotation));
  }

  /**
   * Yaws the current pose left/right
   * 3D rotations can get confusing quick, this will yaw the pose as if it were
   * a plane flying in its starting orientation.
   * @param yaw number of degrees to yaw by
   */
  public void yaw(final double yaw) {
    final Translation3d translation = new Translation3d();
    final Rotation3d startingRotation = this.getRotation();

    // Undo the starting rotation
    this.transformBy(new Transform3d(translation, startingRotation.unaryMinus()));

    // Apply yaw
    final double yawRad = Math.toRadians(yaw);
    final Rotation3d rotation = new Rotation3d(0, 0, yawRad);
    this.transformBy(new Transform3d(translation, rotation));

    // Reapply starting rotation
    this.transformBy(new Transform3d(translation, startingRotation));
  }
}
