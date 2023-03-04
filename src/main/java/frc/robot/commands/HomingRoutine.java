// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmShoulder;
public class HomingRoutine extends CommandBase {

  private final ArmBase armBase;
  private final ArmShoulder armShoulder;
  
  /** Creates a new Homing Routine. */
  public HomingRoutine(ArmBase armBaseSubsystem, ArmShoulder armShoulderSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armBase = armBaseSubsystem;
    this.armShoulder = armShoulderSubsystem;

    addRequirements(armBaseSubsystem, armShoulderSubsystem);

    



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!armBase.baseLimitSwitch()) {
      armBase.changeBaseSpeed(0.5);
    } else {
      armBase.changeBaseSpeed(0);
      armBase.resetBaseEncoder();
    }
    if(!armShoulder.shoulderLimitSwitch()) {
      armShoulder.changeShoulderSpeed(0.5);
    } else {
      armShoulder.changeShoulderSpeed(0);
      armShoulder.resetShoulderEncoder();
    }

    armBase.getBaseQuad();
    armShoulder.getShoulderQuad();
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
