// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToPose;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.ArmKinematics;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.CompilationArm;

public class MoveBaseToPose extends CommandBase {
  private ArmBase armBase;
  private Pose pose;

  /** Creates a new moveBaseToPose. */
  public MoveBaseToPose(Pose pose, ArmBase armBase, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armBase = armBase;
    this.pose = pose;

    addRequirements(armBase, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armBase.moveBase((int) (0.5 + ArmKinematics.getBaseRotation(pose)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armBase.isBaseAtSetpoint();
  }
}