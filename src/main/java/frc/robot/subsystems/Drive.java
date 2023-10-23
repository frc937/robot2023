// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for the drivetrain. Allows both field-oriented and robot-oriented drive. */
public class Drive extends SubsystemBase {

  /* Motor controllers */
  private WPI_TalonSRX frontLeft, frontRight, rearLeft, rearRight;
  private MotorControllerGroup left, right;

  private DifferentialDriveKinematics kinematics;

  private DifferentialDrivePoseEstimator whereTheHeckAreWe;

  private RamseteController ramseteController;

  private DifferentialDrive drivetrain;

  private AHRS gyroscope;

  private LimelightManager limelightManager;

  /** Creates a new drivetrain using IDs from {@link Constants.Drive}. */
  public Drive(LimelightManager limelightManager) {
    /* Instantiates the motor controllers for each mecanum wheel. */
    frontLeft = new WPI_TalonSRX(Constants.Drive.ID_TALON_FRONT_LEFT);
    frontRight = new WPI_TalonSRX(Constants.Drive.ID_TALON_FRONT_RIGHT);
    rearLeft = new WPI_TalonSRX(Constants.Drive.ID_TALON_REAR_LEFT);
    rearRight = new WPI_TalonSRX(Constants.Drive.ID_TALON_REAR_RIGHT);

    /* "Makes the robot not go whee-whee" - Quinn */
    /* In actuality this inverts the right side of the drivetrain, since WPIlib doesn't do that for us anymore.
     * In fairness, "makes the robot not go whee-whee" might actually be an accurate way to describe that.
     */
    frontLeft.setInverted(false);
    rearLeft.setInverted(false);
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    /* Set motor controllers to brake mode */
    frontLeft.setNeutralMode(NeutralMode.Brake);
    rearLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    rearRight.setNeutralMode(NeutralMode.Brake);

    left = new MotorControllerGroup(frontLeft, rearLeft);
    right = new MotorControllerGroup(frontRight, rearRight);

    /* Instantiates the MecanumDrive drivetrain controller. */
    drivetrain = new DifferentialDrive(left, right);

    kinematics = new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);

    /* Instantiates the gyroscope. */
    gyroscope = new AHRS(SPI.Port.kMXP);

    this.limelightManager = limelightManager;

    /* Odometry, has a funny name because funny */
    whereTheHeckAreWe =
        new DifferentialDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation2d(),
            /* Starting encoder dist should be zero always */
            0,
            0,
            /* Initial pose can be 0,0 since it should be set by the first AutoTask */
            new Pose2d());

    ramseteController = new RamseteController();

    SmartDashboard.putString("REMEMBER TO WRITE DOWN YOUR PID VALUES", "I MEAN IT, WRITE THEM DOWN");
    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive I", 0);
    SmartDashboard.putNumber("Drive D", 0);
  }

  /*
   * *looks at commented-out code*
   * *cries in programmer*
   */
  /**
   * Moves the robot in robot-oriented drive mode.
   *
   * <p>All params are -1.0 to 1.0.
   *
   * @param y Robot speed along Y axis, which is left-right. Left is positive.
   * @param x Robot speed along X axis, which is forward-backward. Forward is positive.
   * @param z Robot rotation speed around Z axis. Counterclockwise is positive.
   */
  /*public void moveMecanumRobot(double y, double x, double z) {
    // Sets the default drive mode to Cartesian
    // drivetrain.driveCartesian(x, y, z, ahrs.getAngle());
    drivetrain.driveCartesian(y, x, z);
  }*/

  /**
   * Moves the robot in field-oriented drive mode.
   *
   * <p>All params are -1.0 to 1.0.
   *
   * <p>Keep in mind that "forward" in this context means forward relative to the field, provided
   * that the gyro and everything are working the way they should.
   *
   * @param y Robot speed along Y axis, which is left-right. Left is positive.
   * @param x Robot speed along X axis, which is forward-backward. Forward is positive.
   * @param z Robot rotation speed around Z axis. Counterclockwise is positive.
   */
  /*public void moveMecanumField(double y, double x, double z) {
    drivetrain.driveCartesian(y, x, z, Rotation2d.fromDegrees(gyroscope.getAngle()));
  }*/

  private WPI_TalonSRX configureTalonPID(WPI_TalonSRX talon, double P, double I, double D) {
    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);

    return talon;
  }

  /* For testing ONLY - DO NOT call in production */
  public void updatePIDValuesFromSmartDash() {
    double P = SmartDashboard.getNumber("Drive P", 0);
    double I = SmartDashboard.getNumber("Drive I", 0);
    double D = SmartDashboard.getNumber("Drive D", 0);

    frontLeft = configureTalonPID(frontLeft, P, I, D);
    frontRight = configureTalonPID(frontRight, P, I, D);
    rearLeft = configureTalonPID(rearLeft, P, I, D);
    rearRight = configureTalonPID(rearRight, P, I, D);
  }

  /**
   * Moves the robot in arcade drive mode.
   *
   * <p>All params are -1.0 to 1.0.
   *
   * @param x Robot speed along X axis, which is forward-backward. Forward is positive.
   * @param z Robot rotation speed around Z axis. Counterclockwise is positive.
   */
  public void moveArcade(double x, double z) {
    drivetrain.arcadeDrive(x, z);
  }

  /**
   * Moves the robot in tank drive mode.
   *
   * <p>All params are -1.0 to 1.0.
   *
   * @param left Speed of left side of robot drivetrain. Forward is positive.
   * @param right Speed of right side of robot drivetrain. Forward is positive.
   */
  public void moveTank(double left, double right) {
    drivetrain.tankDrive(left, right);
  }

  public void moveSimple(double leftSpeed, double rightSpeed) {
    frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    rearLeft.set(ControlMode.PercentOutput, leftSpeed);
    frontRight.set(ControlMode.PercentOutput, rightSpeed);
    rearRight.set(ControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Sets the velocity setpoint of the left and right sides of the drivetrain.
   * 
   * <p>These are PID setpoints - they will require the drivetrain to have appropriately-tuned
   * PID gains.
   * @param velocityLeft Velocity setpoint for the left side of the drivetrain
   * @param velocityRight Velocity setpoint for the right side of the drivetrain
   */
  public void setVelocity(double velocityLeft, double velocityRight) {
    frontLeft.set(ControlMode.Velocity, velocityLeft);
    rearLeft.set(ControlMode.Velocity, velocityLeft);
    frontRight.set(ControlMode.Velocity, velocityRight);
    rearRight.set(ControlMode.Velocity, velocityRight);
  }

  /**
   * Stop.
   *
   * <p>(stops all motors controlled by this subsystem)
   */
  public void stop() {
    frontLeft.stopMotor();
    frontRight.stopMotor();
    rearLeft.stopMotor();
    rearRight.stopMotor();
  }

  /**
   * Get the average position of the two encoders on the left side of the drivetrain.
   *
   * <p>This should be pretty close to the actual distance travelled, since the motors & encoders
   * should theoretically travel the same or very similar distances
   *
   * @return Average position of the two encoders on the left side of the drivetrain
   */
  private double getAverageLeftPosition() {
    double averageLeftPosition =
        (frontLeft.getSelectedSensorPosition() + rearLeft.getSelectedSensorPosition()) / 2;
    double averageLeftPositionInches =
        (averageLeftPosition * Constants.Drive.DRIVE_ENCODER_PPR)
            / (Constants.Drive.WHEEL_SIZE_INCHES * Math.PI);
    double averageLeftPositionMeters = Units.inchesToMeters(averageLeftPositionInches);
    return averageLeftPositionMeters;
  }

  /**
   * Get the average position of the two encoders on the right side of the drivetrain.
   *
   * <p>This should be pretty close to the actual distance travelled, since the motors & encoders
   * should theoretically travel the same or very similar distances
   *
   * @return Average position of the two encoders on the right side of the drivetrain
   */
  private double getAverageRightPosition() {
    double averageRightPosition =
        (frontRight.getSelectedSensorPosition() + rearRight.getSelectedSensorPosition()) / 2;
    double averageRightPositionInches =
        (averageRightPosition * Constants.Drive.DRIVE_ENCODER_PPR)
            / (Constants.Drive.WHEEL_SIZE_INCHES * Math.PI);
    double averageRightPositionMeters = Units.inchesToMeters(averageRightPositionInches);
    return averageRightPositionMeters;
  }

  /** Resets the gyroscope. */
  public void resetGyro() {
    gyroscope.reset();
  }

  /**
   * Gets the roll of the robot in degrees
   *
   * @return The roll of the robot
   */
  public double getRoll() {
    return gyroscope.getRoll();
  }

  /**
   * Gets the pitch of the robot in degrees
   *
   * @return The pitch of the robot
   */
  public double getPitch() {
    return gyroscope.getPitch();
  }

  /**
   * This will RESET the pose of the robot - NOT update. It tells the bot "hey, you know all that
   * data you used to have about where the robot is? Ignore ALL OF THAT, this is where you are now."
   *
   * <p>As such, it should be used VERY sparingly. I recommend that it is only run by the AutoTask
   * that zeros field-oriented drive, and MAYBE at the start of teleop if we aren't connected to the
   * field.
   *
   * <p>The pose supplied to it should probably come from the Limelight--the purpose of this method
   * is to make sure that the pose the robot thinks it's at is field-oriented and doesn't place the
   * origin at whatever arbitrary point on the field that we set the bot down at.
   *
   * @param currentPose The pose to reset the bot's odometry to.
   */
  public void resetPosition(Pose2d currentPose) {
    whereTheHeckAreWe.resetPosition(
        gyroscope.getRotation2d(),
        this
            .getAverageLeftPosition(), // we miiiiight need to create an offset and zero these,
                                       // which would be horrible, but doable
        this.getAverageRightPosition(),
        currentPose);
  }

  /**
   * Tracks a provided trajectory.
   *
   * <p>Should be called at a regular interval, like with an execute() method in a command.
   *
   * <p>This thing is confusing as hell, so there should be an example command for how to use it
   * somewhere in this project.
   *
   * @param nextState The trajectory state representing where the robot should be at the time in
   *     tracking the trajectory when this method is called.
   *     <p>One way to do this would be store the value of {@link Timer#getFPGATimestamp()} the
   *     first time this method is called, then to use {@link Trajectory#sample()
   *     Trajectory.sample(Timer.getFPGATimestamp - initialFPGATimestamp)}
   * @param heading The desired heading at the nextState
   */
  public void trackTrajectory(Trajectory.State nextState) {
    ChassisSpeeds chassisSpeeds =
        ramseteController.calculate(whereTheHeckAreWe.getEstimatedPosition(), nextState);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    frontLeft.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond);
    frontRight.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond);
    rearLeft.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond);
    rearRight.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Returns true if the ramseteController is at its reference point, or in other words, done
   * tracking its trajectory
   *
   * @return A boolean; true if the holonomicController done tracking its trajectory
   */
  public boolean trajectoryDone() {
    return ramseteController.atReference();
  }

  /**
   * Subsystem periodic, runs every scheduler loop.
   *
   * <p>Updates odometry (aka whereTheHeckAreWe) with wheel positions and gyro heading, and adds the
   * current vision measurement to odometry's kalman filter.
   */
  @Override
  public void periodic() {
    whereTheHeckAreWe.updateWithTime(
        Timer.getFPGATimestamp(),
        gyroscope.getRotation2d(),
        this.getAverageLeftPosition(),
        this.getAverageRightPosition());
    if (limelightManager.hasValidTarget()) {
      /* This *should* check if the pose from the limelight is within 1m of the current odometry pose,
       * which the odometry recommends we do to prevent us from getting noisy measurements
       */
      if ((Math.abs(
                  limelightManager.getBotpose2d().getX() - whereTheHeckAreWe.getEstimatedPosition().getX())
              >= 1)
          && (Math.abs(
                  limelightManager.getBotpose2d().getY() - whereTheHeckAreWe.getEstimatedPosition().getY())
              >= 1)) {
        whereTheHeckAreWe.addVisionMeasurement(limelightManager.getBotpose2d(), Timer.getFPGATimestamp());
      }
    }

    /* TODO: REMOVE IN PROD */
    updatePIDValuesFromSmartDash();
  }
}
