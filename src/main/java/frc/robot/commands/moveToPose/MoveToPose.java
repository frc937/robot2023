// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.moveToPose;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

/** Add your docs here. */
public final class MoveToPose {
  public MoveToPose() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command extendingMoveToPose(
      Pose pose,
      ArmBase armBase,
      ArmShoulder armShoulder,
      ArmExtender armExtender,
      CompilationArm compilationArm) {
    return Commands.sequence(
        new MoveBaseToPose(pose, armBase, compilationArm),
        new MoveShoulderToPose(pose, armShoulder, compilationArm),
        new MoveExtenderToPose(pose, armExtender, compilationArm));
  }

  public static Command retractingMoveToPose(
      Pose pose,
      ArmBase armBase,
      ArmShoulder armShoulder,
      ArmExtender armExtender,
      CompilationArm compilationArm) {
    return Commands.sequence(
        new MoveExtenderToPose(pose, armExtender, compilationArm),
        new MoveShoulderToPose(pose, armShoulder, compilationArm),
        new MoveBaseToPose(pose, armBase, compilationArm));
  }
}
