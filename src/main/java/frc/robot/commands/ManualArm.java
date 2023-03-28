// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;
import frc.robot.subsystems.arm.ArmExtender;

public class ManualArm extends CommandBase {
  /** Creates a new ManualArm. */
  private final ArmBase armBaseMove;

  private final ArmShoulder armShoulderMove;
  private final ArmExtender armExtenderMove;

  private double armX;
  private double armY;
  private double armZ;

  public ManualArm(ArmBase armBaseSubsystem, ArmShoulder armShoulderSubsystem, ArmExtender armExtenderSubsystem, CompilationArm compilationArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armBaseMove = armBaseSubsystem;
    this.armShoulderMove = armShoulderSubsystem;
    this.armExtenderMove = armExtenderSubsystem;

    addRequirements(armBaseSubsystem, armShoulderSubsystem, compilationArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    armX = RobotContainer.getScaledRightXAxis();
    armY = RobotContainer.getScaledRightYAxis();
    /*armZ = RobotContainer.getScaledLeftYAxis();*/

    //armBaseMove.manualMoveArmBase(armX);
    armShoulderMove.manualMoveArmShoulder(armY);
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
