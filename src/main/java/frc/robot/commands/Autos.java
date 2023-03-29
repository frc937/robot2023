// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.homingRoutine.HomeBase;
import frc.robot.commands.homingRoutine.HomeShoulder;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;


public final class Autos {
  public static CommandBase homingRoutine(ArmShoulder armShoulderSubsystem, ArmBase armBaseSubsystem,ArmExtender armExtenderSubsystem,ArmClaw armClawSubsystem, CompilationArm compilationArmSubsystem) {
    return Commands.sequence(armExtenderSubsystem.setCommand(Constants.Arm.MIN_LENGTH_ARM_EXTENDER), new HomeShoulder(armShoulderSubsystem, compilationArmSubsystem), new HomeBase(armBaseSubsystem, compilationArmSubsystem), armClawSubsystem.openClawCommand());
  }
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
