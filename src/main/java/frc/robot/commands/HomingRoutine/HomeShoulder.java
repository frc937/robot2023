// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HomingRoutine;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

public class HomeShoulder extends CommandBase {
  private final ArmShoulder armShoulder;
  /** Creates a new HomeShoulder. */
  public HomeShoulder(ArmShoulder armShoulder, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armShoulder = armShoulder;

    addRequirements(armShoulder, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armShoulder.manualMoveArmShoulder(-1 * Constants.Arm.SPEED_ARM_SHOULDER_HOMING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armShoulder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armShoulder.shoulderLimitSwitch()) {
      return true;
    } else {
      return false;
    }
  }
}
