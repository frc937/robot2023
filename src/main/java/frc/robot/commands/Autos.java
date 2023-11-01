// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Asmiov's Laws:
 * The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
 * The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
 * The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.homingRoutine.HomeBase;
import frc.robot.commands.homingRoutine.HomeShoulder;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

public final class Autos {
  public static CommandBase homingRoutine(
      ArmShoulder armShoulderSubsystem,
      ArmBase armBaseSubsystem,
      ArmExtender armExtenderSubsystem,
      ArmIntake armClawSubsystem,
      CompilationArm compilationArmSubsystem) {
    return Commands.sequence(
            armExtenderSubsystem.setCommand(Constants.Arm.MIN_LENGTH_ARM_EXTENDER),
            new HomeShoulder(armShoulderSubsystem, compilationArmSubsystem),
            new HomeBase(armBaseSubsystem, compilationArmSubsystem))
        .withTimeout(5.5);
  }

  public static CommandBase homingNoOpenClaw(
      ArmShoulder armShoulder,
      ArmBase armBase,
      ArmExtender armExtender,
      CompilationArm compilationArm) {
    return Commands.sequence(
        armExtender.setCommand(Constants.Arm.MIN_LENGTH_ARM_EXTENDER),
        new HomeShoulder(armShoulder, compilationArm),
        new HomeBase(armBase, compilationArm));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
