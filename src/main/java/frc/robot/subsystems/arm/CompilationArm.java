// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompilationArm extends SubsystemBase {
  private ArmBase armBase;
  private ArmClaw armClaw;
  private ArmExtender armExtender;
  private ArmShoulder armShoulder;


  /** Creates a new CompilationArm. */


  public CompilationArm(ArmBase armBase, ArmClaw armClaw, ArmExtender armExtender, ArmShoulder armShoulder) {
    this.armBase = armBase;
    this.armClaw = armClaw;
    this.armExtender = armExtender;
    this.armShoulder = armShoulder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
