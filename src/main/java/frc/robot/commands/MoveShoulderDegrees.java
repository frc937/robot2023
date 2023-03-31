// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

public class MoveShoulderDegrees extends CommandBase {
  private int degrees;
  private ArmShoulder armShoulder;
  /** Creates a new MoveBaseDegrees. */
  public MoveShoulderDegrees(int degrees, ArmShoulder armShoulder, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.degrees = degrees;
    this.armShoulder = armShoulder;

    addRequirements(armShoulder, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armShoulder.moveShoulder(degrees);
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
    // return armShoulder.isBaseAtSetpoint();
    if (armShoulder.isShoulderAtSetpoint()) {
      System.out.println("Shoulder is supposedly at setpoint");
      return true;
    }
    return false;
  }
}