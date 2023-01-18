// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class AutoTask extends CommandBase {
  /** Creates a new AutoTask. */
  public AutoTask() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
/**
 * Ran when the command is started. Allow basic movements before the PositionSystem moves the robot.
 */
  public abstract void initTask(); 
  /**
   * Checks if the initTask method is finished. Return true when the method finishes.
   * @return the state of the initTask method.
   */
  public abstract boolean initFinished();
  /**
   * Ran when the AutoTask arrives at the defined position.
   */
  public abstract void arrived();
  /**
   * Checks if the Arrived method has finished and allows the next queued command to run.
   * @return the state of the arrived method
   */
  public abstract boolean arrivedFinished();
  /**
   * Ran if the bot cant get to the position it needs.
   * @param positions The bots current position.
   */
  public abstract void fallback(BotPos positions);

  /**
   * <strong>DONT OVERRIDE.</strong> Override update instead.
   */
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
