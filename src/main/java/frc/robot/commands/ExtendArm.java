// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmExtender;

public class ExtendArm extends CommandBase {
  private final ArmExtender armExtender;

  /** Creates a new RetractArm. */
  public ExtendArm(ArmExtender armExtender) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armExtender = armExtender;
    addRequirements(armExtender);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.armExtender.extend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armExtender.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
