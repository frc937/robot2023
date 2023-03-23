// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.I2CManager;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;

/** This subsystem represents the climber in a box that extends the arm on the robot. */
public class ArmExtender extends SubsystemBase {

  private WPI_TalonSRX winch;
  private boolean extenderAtSetpoint;

  private I2CManager I2CManager;

  private Double setpoint;

  /** Creates a new ArmExtender. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmExtender(I2CManager I2CManager) {
    winch = new WPI_TalonSRX(Constants.Arm.ID_TALON_ARM_WINCH);
    setpoint = Constants.Arm.MIN_LENGTH_ARM_EXTENDER;
    this.I2CManager = I2CManager;
    extenderAtSetpoint = false;
  }



  public void Extend() {
    winch.set(ControlMode.Velocity, 0.2);
  }

  public void Retract() {
    winch.set(ControlMode.Velocity, -0.2);
  }

  public Command ExtendCommand() {
    return this.runOnce(() -> this.Extend());
  }

  public Command RetractCommand() {
    return this.runOnce(() -> this.Retract());
  }

  /**
   * If the length sensor has a valid reading, this method will return the length of the arm from
   * the shoulder to the claw.
   *
   * @return The length of the arm from the shoulder to the claw in inches.
   */
  public double getLength() {
    return I2CManager.getCurrentRange(); /* TODO: CHANGE THIS WHEN I2C MANAGER SUBSYSTEM IS DONE */
  }

  /**
   * Sets the setpoint for the arm extension.
   *
   * @param setpoint How far we want the arm to extend in inches from the shoulder to the claw.
   */
  public void set(double setpoint) {
    this.setpoint = setpoint;
  }

  public boolean isExtenderAtSetpoint() {
    return extenderAtSetpoint;
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
          winch.set(-1 * Constants.Arm.SPEED_WINCH_ARM_EXTENSION);
        } else {
          winch.set(Constants.Arm.SPEED_WINCH_ARM_EXTENSION);
        }
      } else {
        winch.stopMotor();
        extenderAtSetpoint = true;
      }
    }
  }

  public void setArmSpeed(double speed) {
    setpoint = null;
    winch.set(speed);
  }
}
