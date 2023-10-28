// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for the drivetrain. (World's most descriptive JavaDoc, you're welcome) */
public class Drive extends SubsystemBase {

  /* Motor controllers */
  private WPI_TalonSRX left, right, frontLeft, frontRight;

  private DifferentialDriveKinematics kinematics;

  private DifferentialDrivePoseEstimator whereTheHeckAreWe;

  private RamseteController ramseteController;

  private DifferentialDrive drivetrain;

  private AHRS gyroscope;

  private LimelightManager limelightManager;

  /** Creates a new drivetrain using IDs from {@link Constants.Drive}. */
  public Drive(LimelightManager limelightManager) {
    /* This is technically the rear left motor controller, but we'll set the front left to follow
     * it, so commanding this controller will command the whole left side of the drivetrain
     */
    left = new WPI_TalonSRX(Constants.Drive.ID_TALON_REAR_LEFT);
    /* Same deal as left */
    right = new WPI_TalonSRX(Constants.Drive.ID_TALON_REAR_RIGHT);
    frontLeft = new WPI_TalonSRX(Constants.Drive.ID_TALON_FRONT_LEFT);
    frontRight = new WPI_TalonSRX(Constants.Drive.ID_TALON_FRONT_RIGHT);

    left.setSensorPhase(true);
    right.setSensorPhase(true);

    /* "Makes the robot not go whee-whee" - Quinn */
    /* In actuality this inverts the right side of the drivetrain, since WPIlib doesn't do that for us anymore.
     * In fairness, "makes the robot not go whee-whee" might actually be an accurate way to describe that.
     */
    left.setInverted(false);
    frontLeft.setInverted(false);
    right.setInverted(true);
    frontRight.setInverted(true);

    /* Set motor controllers to brake mode */
    left.setNeutralMode(NeutralMode.Brake);
    frontLeft.setNeutralMode(NeutralMode.Brake);
    right.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);

    /* Front must follow rear, since DifferentialDrive only takes 2 motor controllers as params */
    frontLeft.follow(left);
    frontLeft.setInverted(InvertType.FollowMaster);
    frontRight.follow(right);
    frontRight.setInverted(InvertType.FollowMaster);

    /* Instantiates the DifferentialDrive drivetrain controller. */
    drivetrain = new DifferentialDrive(left, right);

    /* Kinematics, takes chassis speeds and makes them into wheel speeds and vice versa */
    kinematics = new DifferentialDriveKinematics(Constants.Drive.TRACK_WIDTH);

    /* Instantiates the gyroscope. */
    gyroscope = new AHRS(SPI.Port.kMXP);

    /* Gives us an instance of the Limelight Manager, which chooses which Limelight we pull data
     * from, since there are two on the bot
     */
    this.limelightManager = limelightManager;

    /* Odometry, has a funny name because funny
     * (Theoretically) keeps track of where we are on the field
     */
    whereTheHeckAreWe =
        new DifferentialDrivePoseEstimator(
            kinematics,
            gyroscope.getRotation2d(),
            /* Starting encoder dist should be zero always */
            0,
            0,
            /* Initial pose can be 0,0 since it should be set by the first AutoTask */
            new Pose2d());

    /* Ramsete Controller - tracks trajectories */
    ramseteController = new RamseteController();

    /* Configure PID values for both sides */
    left =
        configureTalonPID(
            left,
            Constants.Drive.DrivePIDYAY.P,
            Constants.Drive.DrivePIDYAY.I,
            Constants.Drive.DrivePIDYAY.D);
    right =
        configureTalonPID(
            right,
            Constants.Drive.DrivePIDYAY.P,
            Constants.Drive.DrivePIDYAY.I,
            Constants.Drive.DrivePIDYAY.D);
    // frontLeft = configureTalonPID(frontLeft, Constants.Drive.DrivePIDYAY.P,
    // Constants.Drive.DrivePIDYAY.I, Constants.Drive.DrivePIDYAY.D);
    // frontRight = configureTalonPID(frontRight, Constants.Drive.DrivePIDYAY.P,
    // Constants.Drive.DrivePIDYAY.I, Constants.Drive.DrivePIDYAY.D);

    /*SmartDashboard.putString("REMEMBER TO WRITE DOWN YOUR PID VALUES", "I MEAN IT, WRITE THEM DOWN");
    SmartDashboard.putNumber("Drive P", 0);
    SmartDashboard.putNumber("Drive I", 0);
    SmartDashboard.putNumber("Drive D", 0);*/
  }

  /** Class to handle converting m/s across the ground to encoder ticks/100ms */
  private static class EvilUnitConverter {
    /**
     * Convert m/s across the ground to encoder ticks/100ms (around the axle)
     *
     * @param metersPerSecond m/s across the ground
     * @return Encoder ticks/100ms (angular momentum around the axle)
     */
    public static double metersPerSecondToEncoderTicksPer100ms(double metersPerSecond) {
      /* This eats memory for breakfast but looks non-gross */
      double metersPer100ms = metersPerSecond / 10;
      double rotationsPer100ms = metersPer100ms / (Constants.Drive.WHEEL_DIAMETER_METERS * Math.PI);
      double ticksPer100ms = rotationsPer100ms * Constants.Drive.DRIVE_ENCODER_PPR;
      return ticksPer100ms;
    }

    /**
     * Convert encoder ticks/100ms (around the axle) to m/s across the ground
     *
     * @param encoderTicksPer100ms Encoder ticks/100ms (angular momentum around the axle)
     * @return m/s across the ground
     */
    public static double encoderTicksPer100msToMetersPerSecond(double encoderTicksPer100ms) {
      double rotationsPer100ms = encoderTicksPer100ms / Constants.Drive.DRIVE_ENCODER_PPR;
      double metersPer100ms = rotationsPer100ms * (Constants.Drive.WHEEL_DIAMETER_METERS * Math.PI);
      double metersPerSecond = metersPer100ms * 10;
      return metersPerSecond;
    }
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

  /**
   * Configures a passed WPI_TalonSRX with the passed PID values
   *
   * @param talon WPI_TalonSRX to configure
   * @param P kP value to set
   * @param I kI value to set
   * @param D kD value to set
   * @return The configured WPI_TalonSRX. You will NEED to set whatever variable represents your
   *     Talon to this returned value, like so: <code>talon = configureTalonPID(talon, kP, kI, kD);
   *     </code>.
   */
  private WPI_TalonSRX configureTalonPID(WPI_TalonSRX talon, double P, double I, double D) {
    /* TODO: change this to configure the whole talon & just use Constants instead of taking params */
    talon.config_kP(0, P);
    talon.config_kI(0, I);
    talon.config_kD(0, D);

    talon.configOpenloopRamp(Constants.Drive.OPEN_LOOP_RAMP_RATE);

    return talon;
  }

  /* For testing ONLY - DO NOT call in production */
  /* void updatePIDValuesFromSmartDash() {
    double P = SmartDashboard.getNumber("Drive P", 0);
    double I = SmartDashboard.getNumber("Drive I", 0);
    double D = SmartDashboard.getNumber("Drive D", 0);

    left = configureTalonPID(left, P, I, D);
    right = configureTalonPID(right, P, I, D);
    frontLeft = configureTalonPID(frontLeft, P, I, D);
    frontRight = configureTalonPID(frontRight, P, I, D);
  }*/

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

  /**
   * Sets the velocity setpoint of the left and right sides of the drivetrain.
   *
   * <p>These are PID setpoints - they will require the drivetrain to have appropriately-tuned PID
   * gains.
   *
   * @param velocityLeft Velocity setpoint for the left side of the drivetrain
   * @param velocityRight Velocity setpoint for the right side of the drivetrain
   */
  public void setVelocity(double velocityLeft, double velocityRight) {
    left.set(ControlMode.Velocity, velocityLeft);
    right.set(ControlMode.Velocity, velocityRight);
  }

  /**
   * Stop.
   *
   * <p>(stops all motors controlled by this subsystem)
   *
   * <p><strong>WILL NOT stop motors that currently have a PID setpoint. Use {@link #setVelocity}
   * and zero their setpoints instead.
   */
  public void stop() {
    left.stopMotor();
    right.stopMotor();
  }

  /**
   * Get the position of the encoder on the left side of the drivetrain.
   *
   * @return Position of the encoder on the left side of the drivetrain
   */
  private double getLeftPosition() {
    double leftPositionMeters =
        EvilUnitConverter.encoderTicksPer100msToMetersPerSecond(left.getSelectedSensorPosition());
    return leftPositionMeters;
  }

  /**
   * Get the position of the encoder on the right side of the drivetrain.
   *
   * @return Position of the encoder on the right side of the drivetrain
   */
  private double getRightPosition() {
    double rightPositionMeters =
        EvilUnitConverter.encoderTicksPer100msToMetersPerSecond(right.getSelectedSensorPosition());
    return rightPositionMeters;
  }

  /** Resets the gyroscope to zero degrees. */
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
        this.getLeftPosition(), // we miiiiight need to create an offset and zero these,
        // which would be horrible, but doable
        // I think we can actually just run a method that tells the
        // motor controllers to zero them
        this.getRightPosition(),
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
   */
  public void trackTrajectory(Trajectory.State nextState) {
    ChassisSpeeds chassisSpeeds =
        ramseteController.calculate(whereTheHeckAreWe.getEstimatedPosition(), nextState);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    double leftSetpoint =
        EvilUnitConverter.metersPerSecondToEncoderTicksPer100ms(wheelSpeeds.leftMetersPerSecond);
    double rightSetpoint =
        EvilUnitConverter.metersPerSecondToEncoderTicksPer100ms(wheelSpeeds.rightMetersPerSecond);
    left.set(ControlMode.Velocity, leftSetpoint);
    right.set(ControlMode.Velocity, rightSetpoint);
  }

  /**
   * Returns true if the ramseteController is at its reference point, or in other words, done
   * tracking its trajectory
   *
   * @return A boolean; true if the ramseteController done tracking its trajectory
   */
  public boolean trajectoryDone() {
    return ramseteController.atReference();
  }

  /**
   * @deprecated debug method
   */
  @Deprecated
  public void printPoses() {
    /* KILL THIS IN PROD */
    System.out.println("LL pose: " + limelightManager.getTargetedLimelight().getBotpose2d());
    System.out.println("Odo pose: " + whereTheHeckAreWe.getEstimatedPosition());
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
        this.getLeftPosition(),
        this.getRightPosition());
    Limelight limelight = limelightManager.getTargetedLimelight();
    if (limelight != null) {
      /* This *should* check if the pose from the limelight is within 1m of the current odometry pose,
       * which the odometry recommends we do to prevent us from getting noisy measurements
       */
      if ((Math.abs(
                  limelight.getBotpose2d().getX() - whereTheHeckAreWe.getEstimatedPosition().getX())
              >= 1)
          && (Math.abs(
                  limelight.getBotpose2d().getY() - whereTheHeckAreWe.getEstimatedPosition().getY())
              // nice
              >= 1)) {
        whereTheHeckAreWe.addVisionMeasurement(limelight.getBotpose2d(), Timer.getFPGATimestamp());
      }
    }
  }
}
