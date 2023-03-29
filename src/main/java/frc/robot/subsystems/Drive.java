// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for the drivetrain. Allows both field-oriented and robot-oriented drive. */
public class Drive extends SubsystemBase {

  /* Motor controllers */
  WPI_TalonSRX frontLeft;
  WPI_TalonSRX frontRight;
  WPI_TalonSRX rearLeft;
  WPI_TalonSRX rearRight;

  MecanumDrive drivetrain;

  AHRS gyroscope;

  /** Creates a new drivetrain using IDs from {@link Constants.DriveConstants}. */
  public Drive() {
    /* Instantiates the motor controllers for each mecanum wheel. */
    frontLeft = new WPI_TalonSRX(Constants.DriveConstants.ID_TALON_FRONT_LEFT);
    frontRight = new WPI_TalonSRX(Constants.DriveConstants.ID_TALON_FRONT_RIGHT);
    rearLeft = new WPI_TalonSRX(Constants.DriveConstants.ID_TALON_REAR_LEFT);
    rearRight = new WPI_TalonSRX(Constants.DriveConstants.ID_TALON_REAR_RIGHT);

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

    /* Instantiates the gyroscope. */
    gyroscope = new AHRS(SPI.Port.kMXP);
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
}
