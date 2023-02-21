// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

public class MoveToPose extends CommandBase {
  /** Creates a new MoveToPose. */
  private Pose armPose;

  private ArmBase armBase;
  private ArmShoulder armShoulder;
  private ArmExtender armExtender;
  private CompilationArm compilationArm;

  private Pose pose;

  public MoveToPose(
      Pose pose,
      ArmShoulder armShoulder,
      ArmBase armBase,
      ArmExtender armExtender,
      CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(armShoulder, armExtender, armBase, compilationArm);
    this.pose = pose;
    this.compilationArm = compilationArm;
    this.armBase = armBase;
    this.armExtender = armExtender;
    this.armShoulder = armShoulder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    compilationArm.moveToPose(pose);
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
    return false;
  }
}
