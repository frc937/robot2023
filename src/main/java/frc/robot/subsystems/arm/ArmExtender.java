// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** This subsystem represents the climber in a box that extends the arm on the robot. */
public class ArmExtender extends SubsystemBase {

  private WPI_TalonSRX winch;

  /* If VS Code thinks this library can't be found, it's probably wrong.
   * Usually the code builds just fine even if VS Code thinks the library can't be found.
   */
  private Rev2mDistanceSensor lengthSensor;

  private double setpoint;

  /** Creates a new ArmExtender. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmExtender() {
    winch = new WPI_TalonSRX(Constants.Arm.ID_TALON_ARM_WINCH);
    // we do not know which stage corresponds to which port, so change later
    lengthSensor = new Rev2mDistanceSensor(Port.kOnboard);
    lengthSensor.setAutomaticMode(true);
    setpoint = Constants.Arm.MIN_LENGTH_ARM_EXTENDER;
  }

  /**
   * If the length sensor has a valid reading, this method will return the length of the arm from
   * the shoulder to the claw.
   *
   * @return The length of the arm from the shoulder to the claw in inches.
   */
  public double getLength() {
    if (lengthSensor.isRangeValid()) {
      return lengthSensor.getRange();
    } else {
      return -1.0;
    }
  }

  /**
   * Sets the setpoint for the arm extension.
   *
   * @param setpoint How far we want the arm to extend in inches from the shoulder to the claw.
   */
  public void set(double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Directs arm towards setpoint. Since this is the periodic method, this is called every time the
   * scheduler runs.
   */
  @Override
  public void periodic() {
    /* Adds a tolerance so we don't vibrate back and forth constantly and destroy the entire mechanism */
    if (Math.abs(setpoint - getLength()) >= Constants.Arm.DONE_THRESHOLD_ARM_EXTENSION) {
      if (getLength() > setpoint) {
        winch.set(-1 * Constants.Arm.SPEED_WINCH_ARM_EXTENSION);
      } else {
        winch.set(Constants.Arm.SPEED_WINCH_ARM_EXTENSION);
      }
    } else {
      winch.stopMotor();
    }
  }
}
