// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that represents the claw on the arm. Can open the claw or close it to a given pressure.
 */
public class ArmClaw extends SubsystemBase {

  private Talon clawMotor;
  private Double setpoint;

  /*
   * *********************************************************************************
   * TODO: REPLACE THIS WITH A REAL PRESSURE SENSOR OBJECT BEFORE WE USE IT ON THE BOT
   * *********************************************************************************
   */
  private AnalogInput pressure;

  /** Creates a new ArmClaw. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmClaw() {
    clawMotor = new Talon(Constants.Arm.ID_TALON_ARM_CLAW);
    pressure = new AnalogInput(Constants.Arm.CHANNEL_ANALOG_PRESSURE_SENSOR);
    /* We set the setpoint to null when we want to disable the code that automagically moves the claw to the setpoint */
    setpoint = null;
  }

  /** Opens the claw. */
  public void openClaw() {
    /* We set the setpoint to null when we want to disable the code that automagically moves the claw to the setpoint */
    setpoint = null;
    clawMotor.set(-1 * Constants.Arm.SPEED_ARM_CLAW);
  }

  public void manualCloseClaw() {
    setpoint = null;
    clawMotor.set(Constants.Arm.SPEED_ARM_CLAW);
  }

  public void stop() {
    clawMotor.set(0);
  }

  /**
   * Command factory to open the claw
   *
   * @return a command that opens the claw
   */
  public Command openClawCommand() {
    return this.runOnce(() -> this.openClaw());
  }

  public Command manualOpenClawCommand() {
    return this.runEnd(() -> this.openClaw(), () -> this.stop());
  }

  public Command manualCloseClawCommand() {
    return this.runEnd(() -> this.manualCloseClaw(), () -> this.stop());
  }

  /* TODO: Determine what the pressure sensor's gonna be from mechanical, and, therefore, what units it will use. */
  /**
   * Sets the pressure-based setpoint for the claw.
   *
   * @param setpoint How much pressure we want the claw to apply to whatever it's clamping onto. Units are in volts of resistance, least resistance is 5v, most is 0. Least resistence = most pressure.
   */
  public void set(Double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Subsystem periodic; called once per scheduler run.
   *
   * <p>Moves the claw to the current setpoint.
   */
  @Override
  public void periodic() {
    /* Allows us to have a way to make this code not run, so we can do things like open the claw. */
    if (setpoint == null) {
      return;
    } else {
      /* Adds a tolerance so we don't vibrate back and forth constantly and destroy the entire mechanism */
      if (Math.abs(setpoint - pressure.getVoltage()) >= Constants.Arm.DONE_THRESHOLD_ARM_CLAW) {
        if (pressure.getVoltage() > setpoint) {
          clawMotor.set(-1 * Constants.Arm.SPEED_ARM_CLAW);
        } else {
          clawMotor.set(Constants.Arm.SPEED_ARM_CLAW);
        }
      }
    }
  }
}
