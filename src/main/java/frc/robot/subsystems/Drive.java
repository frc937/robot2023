// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for the drivetrain. Allows both field-oriented and robot-oriented drive. */
public class Drive extends SubsystemBase {

  /* Motor controllers */
  private WPI_TalonSRX frontLeft, frontRight, rearLeft, rearRight;

  private MecanumDriveKinematics kinematics;

  private MecanumDrivePoseEstimator whereTheHeckAreWe;

  private HolonomicDriveController holonomicController;

  private MecanumDrive drivetrain;

  private AHRS gyroscope;

  private Limelight limelight;

  /** Creates a new drivetrain using IDs from {@link Constants.Drive}. */
  public Drive(Limelight limelight) {
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

    /* Instantiates the MecanumDrive drivetrain controller. */
    drivetrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    kinematics =
        new MecanumDriveKinematics(
            Constants.Drive.LOCATION_WHEEL_FRONT_LEFT,
            Constants.Drive.LOCATION_WHEEL_FRONT_RIGHT,
            Constants.Drive.LOCATION_WHEEL_REAR_LEFT,
            Constants.Drive.LOCATION_WHEEL_REAR_RIGHT);

    /* Instantiates the gyroscope. */
    gyroscope = new AHRS(SPI.Port.kMXP);

    this.limelight = limelight;

    /* Odometry, has a funny name because funny */
    whereTheHeckAreWe =
        new MecanumDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation2d(),
            new MecanumDriveWheelPositions(
                /* This is proooobably the right way to do this */
                frontLeft.getSelectedSensorPosition(),
                frontRight.getSelectedSensorPosition(),
                rearLeft.getSelectedSensorPosition(),
                rearRight.getSelectedSensorPosition()),
            /* Initial pose can be 0,0 since it should be set by the first AutoTask */
            new Pose2d());

    /* CONSTANTS OUT THE BUTTHOLE */
    holonomicController =
        new HolonomicDriveController(
            new PIDController(
                Constants.Drive.HolonomicController.XController.P,
                Constants.Drive.HolonomicController.XController.I,
                Constants.Drive.HolonomicController.XController.D),
            new PIDController(
                Constants.Drive.HolonomicController.YController.P,
                Constants.Drive.HolonomicController.YController.I,
                Constants.Drive.HolonomicController.YController.D),
            new ProfiledPIDController(
                Constants.Drive.HolonomicController.ThetaController.P,
                Constants.Drive.HolonomicController.ThetaController.I,
                Constants.Drive.HolonomicController.ThetaController.D,
                new TrapezoidProfile.Constraints(
                    Constants.Drive.HolonomicController.ThetaController.Constraints.MAX_VELOCITY,
                    Constants.Drive.HolonomicController.ThetaController.Constraints
                        .MAX_ACCELERATION)));
  }

  /**
   * Moves the robot in robot-oriented drive mode.
   *
   * <p>All params are -1.0 to 1.0.
   *
   * @param y Robot speed along Y axis, which is left-right. Left is positive.
   * @param x Robot speed along X axis, which is forward-backward. Forward is positive.
   * @param z Robot rotation speed around Z axis. Counterclockwise is positive.
   */
  public void moveMecanumRobot(double y, double x, double z) {
    /* Sets the default drive mode to Cartesian */
    // drivetrain.driveCartesian(x, y, z, ahrs.getAngle());
    drivetrain.driveCartesian(y, x, z);
  }

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
  public void moveMecanumField(double y, double x, double z) {
    drivetrain.driveCartesian(y, x, z, Rotation2d.fromDegrees(gyroscope.getAngle()));
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
   * Resets the gyroscope, such that whatever direction the robot is pointed at the time is now
   * deemed "foward" in field-oriented drive mode
   */
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

  public MecanumDrivePoseEstimator getMecanumDrivePoseEstimator() {
    return whereTheHeckAreWe;
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
        new MecanumDriveWheelPositions(
            frontLeft.getSelectedSensorPosition(),
            frontRight.getSelectedSensorPosition(),
            rearLeft.getSelectedSensorPosition(),
            rearRight.getSelectedSensorPosition()),
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
  public void trackTrajectory(Trajectory.State nextState, Rotation2d heading) {
    ChassisSpeeds chassisSpeeds =
        holonomicController.calculate(whereTheHeckAreWe.getEstimatedPosition(), nextState, heading);
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    frontLeft.set(ControlMode.Velocity, wheelSpeeds.frontLeftMetersPerSecond);
    frontRight.set(ControlMode.Velocity, wheelSpeeds.frontRightMetersPerSecond);
    rearLeft.set(ControlMode.Velocity, wheelSpeeds.rearLeftMetersPerSecond);
    rearRight.set(ControlMode.Velocity, wheelSpeeds.rearRightMetersPerSecond);
  }

  /**
   * Returns true if the holonomicController is at its reference point, or in other words, done
   * tracking its trajectory
   *
   * @return A boolean; true if the holonomicController done tracking its trajectory
   */
  public boolean trajectoryDone() {
    return holonomicController.atReference();
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
        new MecanumDriveWheelPositions(
            frontLeft.getSelectedSensorPosition(),
            frontRight.getSelectedSensorPosition(),
            rearLeft.getSelectedSensorPosition(),
            rearRight.getSelectedSensorPosition()));
    if (limelight.hasValidTarget()) {
      /* This *should* check if the pose from the limelight is within 1m of the current odometry pose,
       * which the odometry recommends we do to prevent us from getting noisy measurements
       */
      if ((Math.abs(
                  limelight.getBotpose2d().getX() - whereTheHeckAreWe.getEstimatedPosition().getX())
              >= 1)
          && (Math.abs(
                  limelight.getBotpose2d().getY() - whereTheHeckAreWe.getEstimatedPosition().getY())
              >= 1)) {
        whereTheHeckAreWe.addVisionMeasurement(limelight.getBotpose2d(), Timer.getFPGATimestamp());
      }
    }
  }
}
