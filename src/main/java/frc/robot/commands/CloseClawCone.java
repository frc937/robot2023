// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmClaw;

public class CloseClawCone extends CommandBase {
  private ArmClaw armClaw;

  /** Creates a new CloseClawCone. */
  public CloseClawCone(ArmClaw armClaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armClaw = armClaw;

    addRequirements(armClaw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armClaw.set(Constants.Arm.SETPOINT_PRESSURE_CONE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armClaw.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armClaw.isAtSetpoint();
  }
}
