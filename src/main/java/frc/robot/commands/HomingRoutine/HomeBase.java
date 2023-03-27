// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HomingRoutine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.CompilationArm;

public class HomeBase extends CommandBase {
  private final ArmBase armBase;
  /** Creates a new HomeShoulder. */
  public HomeBase(ArmBase armShoulder, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armBase = armShoulder;

    addRequirements(armShoulder, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armBase.manualMoveArmBase(Constants.Arm.SPEED_ARM_BASE_HOMING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armBase.baseLimitSwitch()) {
      return true;
    } else {
      return false;
    }
  }
}
