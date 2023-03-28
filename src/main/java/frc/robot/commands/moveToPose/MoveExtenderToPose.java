// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToPose;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.ArmKinematics;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.CompilationArm;

public class MoveExtenderToPose extends CommandBase {
  private ArmExtender armExtender;
  private Pose pose;

  /** Creates a new moveBaseToPose. */
  public MoveExtenderToPose(Pose pose, ArmExtender armExtender, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armExtender = armExtender;
    this.pose = pose;

    addRequirements(armExtender, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armExtender.set(ArmKinematics.getArmExtension(pose));
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
    return armExtender.isExtenderAtSetpoint();
  }
}
