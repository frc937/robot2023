// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.autotasks.base.BotPos;

public abstract class AutoTask extends CommandBase {
  /** Creates a new AutoTask. */
  public AutoTask() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public abstract void initTask(); 
  public abstract boolean initFinished();
  public abstract void arrived();
  public abstract boolean arrivedFinished();

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  
  public abstract void update(BotPos position);
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
