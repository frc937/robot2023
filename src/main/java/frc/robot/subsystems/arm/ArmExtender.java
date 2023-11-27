// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Asimov's Laws:
 * The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
 * The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
 * The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.
 */
package frc.robot.subsystems.arm;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.I2CManager;

/** This subsystem represents the climber in a box that extends the arm on the robot. */
public class ArmExtender extends SubsystemBase {

  private Talon winch;
  private boolean extenderAtSetpoint;

  private I2CManager I2CManager;

  private SlewRateLimiter rateLimiter;

  private Double setpoint;
  private Double percentOutput;

  /** Creates a new ArmExtender. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmExtender(I2CManager I2CManager) {
    winch = new Talon(Constants.Arm.ID_TALON_ARM_WINCH);
    rateLimiter = new SlewRateLimiter(Constants.Arm.RAMP_RATE_WINCH_ARM_EXTENSION);
    // winch.setInverted(Constants.Arm.INVERTED_TALON_ARM_EXTENDER);
    setpoint = null;
    percentOutput = null;
    this.I2CManager = I2CManager;
    extenderAtSetpoint = false;
  }

  public void extend() {
    setpoint = null;
    percentOutput = Constants.Arm.SPEED_WINCH_ARM_EXTENSION;
  }

  public void retract() {
    setpoint = null;
    percentOutput = -1 * Constants.Arm.SPEED_WINCH_ARM_EXTENSION;
  }

  public Command extendCommand() {
    return this.runOnce(() -> this.extend());
  }

  public Command retractCommand() {
    return this.runOnce(() -> this.retract());
  }

  /**
   * If the length sensor has a valid reading, this method will return the length of the arm from
   * the shoulder to the claw.
   *
   * @return The length of the arm from the shoulder to the claw in inches.
   */
  public double getLength() {
    return I2CManager.getCurrentRange() + Constants.Arm.EXTRA_LENGTH_ARM_EXTENDER;
  }

  /**
   * Sets the setpoint for the arm extension.
   *
   * @param setpoint How far we want the arm to extend in inches from the shoulder to the claw.
   */
  public void set(double setpoint) {
    percentOutput = null;
    setpoint -= Constants.Arm.EXTRA_LENGTH_ARM_EXTENDER;
    this.setpoint = setpoint;
  }

  public Command setCommand(double setpoint) {
    return this.runOnce(() -> this.set(setpoint));
  }

  public boolean isExtenderAtSetpoint() {
    return extenderAtSetpoint;
  }

  private double extenderCompensatedFeedForward(ArmExtender armExtender) {
    if (armExtender.getLength() >= )
  }

  /**
   * Directs arm towards setpoint. Since this is the periodic method, this is called every time the
   * scheduler runs.
   */
  @Override
  public void periodic() {
    if (setpoint != null && I2CManager.isCurrentRangeValid()) {
      /* Adds a tolerance so we don't vibrate back and forth constantly and destroy the entire mechanism */
      if (Math.abs(setpoint - getLength()) >= Constants.Arm.DONE_THRESHOLD_ARM_EXTENSION) {
        if (getLength() > setpoint) {
          winch.set(rateLimiter.calculate(-1 * Constants.Arm.SPEED_WINCH_ARM_EXTENSION));
        } else {
          winch.set(rateLimiter.calculate(Constants.Arm.SPEED_WINCH_ARM_EXTENSION));
        }
      } else {
        winch.stopMotor();
        extenderAtSetpoint = true;
      }
    }

    if (percentOutput != null) {
      winch.set(rateLimiter.calculate(percentOutput));
    }

    System.out.println("Extender length: " + getLength());
  }

  public void setArmSpeed(double speed) {
    setpoint = null;
    percentOutput = speed;
  }
}
