// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class AutoTask extends CommandBase {
  private boolean initialized = false;
  private Position taskPos;
  /** Creates a new AutoTask. <p>
   * Dont forgot to use addRequirements() to add subsystem dependencies.
   */
  public AutoTask() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    initTask();
  }
/**
 * Ran when the command is started. Allow basic movements before the PositionSystem moves the robot. <p>
 * Make sure to put the taskPosition() method in init to set where the robot will go. <p>
 * Not running taskPosition() will cause a NullPointerException
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
  public abstract void fallback(Position positions);

  /**
   * <strong>DONT OVERRIDE.</strong> Override update instead.
   */
  @Override
  public void execute() {
    if (initFinished() & !initialized){
      //code will be put here when the PositionSystem is implimented 
      if (taskPos == null){ // checks if taskpos was instantiated and if not throw an error
        throw new NullPointerException("taskPositon Was not ran in initTask.");
      }
    }
    
  }
  /**
   * Use instead of execute. Functions as execute but with 
   * @param position is the current position of the robot when update is ran.
   */
  public abstract void update(Position position);
  /**
   * Sets the position the bot will go to for the task. <p>
   * Not running this in initCommand <strong>will</strong> cause a NullPointerException
   * @param position The position the bot will go to for the AutoTask
   */
  protected void taskPosition(Position position){
    taskPos = position;
  }
  @Override
  public void end(boolean interrupted) {} 


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
