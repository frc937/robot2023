// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.HomingRoutine;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;


public final class Autos {
  public static CommandBase homingRoutine(ArmShoulder armShoulderSubsystem, ArmBase armBaseSubsystem,ArmExtender armExtenderSubsystem,ArmClaw armClawSubsystem ) {
    return Commands.sequence(new HomingRoutine(armBaseSubsystem, armShoulderSubsystem,armExtenderSubsystem,armClawSubsystem));
  }
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
