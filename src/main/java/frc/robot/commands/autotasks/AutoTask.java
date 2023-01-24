// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autotasks;

import frc.robot.positioning.Position;
public abstract class AutoTask {
  private boolean initialized = false;
  private Position taskPos;
  /** Creates a new AutoTask. <p>
   * Dont forgot to use to add subsystem dependencies.
   */
  public AutoTask() {
  }

  
  public void initialize() {
    initTask();
  }
/**
 * Ran when the command is started. Allow basic movements before the PositionSystem moves the robot. <p>
 * Make sure to put the {@link #taskPosition(Position) taskPosition} method in init to set where the robot will go. <p>
 * Not running taskPosition will cause a NullPointerException.
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
   * @param position The bots current position.
   */
  public abstract void fallback(Position position);

  /**
   * <strong>DONT OVERRIDE.</strong> Override update instead. Overriding this will cause the AutoTask to never start.
   */
  public void execute() {
    if (initFinished() & !initialized){
      //code will be put here when the PositionSystem is implimented 
      if (taskPos == null){ // checks if taskpos was instantiated and if not throw an error
        throw new NullPointerException("taskPositon Was not ran in initTask.");
      }
    }
    
  }
  /**
   * Use instead of execute. Functions as execute but with a position arguemnt.
   * @param position the current position of the robot when update is ran.
   */
  public abstract void update(Position position);
  /**
   * Sets the position the bot will go to for the task. <p>
   * Not running this in {@link #initTask initTask} <strong>will</strong> cause a NullPointerException
   * @param position The position the bot will go to for the AutoTask
   */
  protected void taskPosition(Position position){
    taskPos = position;
  }
  public void end(boolean interrupted) {} 


  // Returns true when the command should end.
  public boolean isFinished() {
    return false;
  }
}
