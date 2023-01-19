// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that represents the claw on the arm. Can open the claw or close it to a given pressure.
 */
public class ArmClaw extends SubsystemBase {

  private WPI_TalonSRX clawMotor;
  private Double setpoint;

  /* 
   * *********************************************************************************
   * TODO: REPLACE THIS WITH A REAL PRESSURE SENSOR OBJECT BEFORE WE USE IT ON THE BOT
   * *********************************************************************************
   */
  private double clawPressure;

  /** 
   * Creates a new ArmClaw. Should be called once from {@link frc.robot.RobotContainer}.
   */
  public ArmClaw() {
    clawMotor = new WPI_TalonSRX(Constants.ArmConstants.ID_TALON_ARM_CLAW);
    /* We set the setpoint to null when we want to disable the code that automagically moves the claw to the setpoint */
    setpoint = null;
  }

  /**
   * Opens the claw.
   */
  public void openClaw() {
    /* We set the setpoint to null when we want to disable the code that automagically moves the claw to the setpoint */
    setpoint = null;
    clawMotor.set(Constants.ArmConstants.SPEED_ARM_CLAW);
  }

  /* TODO: Determine what the pressure sensor's gonna be from mechanical, and, therefore, what units it will use. */
  /**
   * Sets the pressure-based setpoint for the claw.
   * @param setpoint How much pressure we want the claw to apply to whatever it's clamping onto. <b>(UNITS TBD)</b>
   */
  public void set(Double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Subsystem periodic; called once per scheduler run.
   * <p>Moves the claw to the current setpoint.
   */
  @Override
  public void periodic() {
    /* Allows us to have a way to make this code not run, so we can do things like open the claw. */
    if (setpoint.equals(null)) {
      return;
    } else {
      /* Adds a tolerance so we don't vibrate back and forth constantly and destroy the entire mechanism */
      if (Math.abs(setpoint - clawPressure) >= Constants.ArmConstants.DONE_THRESHOLD_ARM_CLAW) {
        if (clawPressure > setpoint) {
          clawMotor.set(-1 * Constants.ArmConstants.SPEED_ARM_CLAW);
        } else {
          clawMotor.set(Constants.ArmConstants.SPEED_ARM_CLAW);
        }
      }
    }
  }
}
