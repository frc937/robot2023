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

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that represents the claw on the arm. Can open the claw or close it to a given pressure.
 */
public class ArmIntake extends SubsystemBase {

  private final Talon clawMotor;

  /** Creates a new ArmClaw. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmIntake() {
    clawMotor = new Talon(Constants.Arm.Intake.ID_TALON_ARM_CLAW);
  }

  /** Stops the arm intake from spinning */
  public void stop() {
    clawMotor.set(0);
  }

  /**
   * Sets the speed the intake runs at.
   *
   * @param setpoint the percent power to run the arm at
   */
  public void set(Double setpoint) {
    clawMotor.set(setpoint);
  }

  public boolean isAtSetpoint() {
    return true;
  }

  /**
   * Subsystem periodic; called once per scheduler run.
   *
   * <p>Moves the claw to the current setpoint.
   */
  @Override
  public void periodic() {}
}
