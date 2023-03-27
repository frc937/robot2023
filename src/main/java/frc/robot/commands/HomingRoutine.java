// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
public class HomingRoutine extends CommandBase {

  private final ArmBase armBase;
  private final ArmShoulder armShoulder;
  private final ArmExtender armExtender;
  private final ArmClaw armClaw;
  
  /** Creates a new Homing Routine. */
  public HomingRoutine(ArmBase armBaseSubsystem, ArmShoulder armShoulderSubsystem, ArmExtender armExtender, ArmClaw armClaw) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armBase = armBaseSubsystem;
    this.armShoulder = armShoulderSubsystem;
    this.armExtender = armExtender;
    this.armClaw = armClaw;

    addRequirements(armBaseSubsystem, armShoulderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armClaw.openClaw();
    armExtender.set(Constants.Arm.MIN_LENGTH_ARM_EXTENDER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!armBase.baseLimitSwitch()) {
      armBase.manualMoveArmBase(0.1);
    } else {
      armBase.manualMoveArmBase(0);
      /* Base will automatically zero its encoder if the reverse limit switch is tripped */
    }
    if(!armShoulder.shoulderLimitSwitch()) {
      armShoulder.manualMoveArmShoulder(0.1);
    } else {
      armShoulder.manualMoveArmShoulder(0);
      /* Shoulder will also automatically zero its encoder if the reverse limit switch is tripped
      */
    }
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
