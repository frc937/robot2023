// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that represents the claw on the arm. Can open the claw or close it to a given pressure.
 */
public class ArmIntake extends SubsystemBase {

  private final Talon clawMotor;

  /** Creates a new ArmClaw. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmIntake() {
    clawMotor = new Talon(Constants.Arm.ID_TALON_ARM_CLAW);
  }

  /** Opens the claw. */
  public void openClaw() {}

  public void manualCloseClaw() {}

  public void stop() {
    clawMotor.set(0);
  }

  /**
   * Command factory to open the claw
   *
   * @return a command that opens the claw
   */
  public Command openClawCommand() {
    return new InstantCommand();
  }

  public Command manualOpenClawCommand() {
    return new InstantCommand();
  }

  public Command manualCloseClawCommand() {
    return new InstantCommand();
  }

  /**
   * Sets the pressure-based setpoint for the claw.
   *
   * @param setpoint How much pressure we want the claw to apply to whatever it's clamping onto.
   *     Units are in volts of resistance, least resistance is 5v, most is 0. Least resistence =
   *     most pressure.
   */
  public void set(Double setpoint) {}

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
