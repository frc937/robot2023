// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autotasks;

import java.util.ArrayList;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.Pose;

public abstract class AutoTask {
  private boolean initialized = false;
  private Pose taskPos;
  private ArrayList<CommandBase> commands = new ArrayList<CommandBase>();
  private Stack<CommandBase> initCommands = new Stack<CommandBase>();
  private Stack<CommandBase> arrivedCommands = new Stack<CommandBase>();
  private CommandBase runningCommand;

  /**
   * Creates a new AutoTask.
   */
  public AutoTask() {
  }

  /**
   * Ran when the command is started. Allow basic movements before the
   * PositionSystem moves the robot.
   * <p>
   * Make sure to put the {@link #setTaskPosition(Position) setTaskPosition}
   * method in
   * init to set where the robot will go.
   * <p>
   * Not running taskPosition will cause a NullPointerException.
   */
  public abstract void initTask();

  /**
   * Checks if the initTask method is finished. Return true when the method
   * finishes.
   * 
   * @return the state of the initTask method.
   */
  public boolean initFinished(){
    if (initCommands.empty() & runningCommand.isFinished()) {
        return true;
    } else {
      return false;
    }
  }

  /**
   * Ran when the AutoTask arrives at the defined position.
   */
  public abstract void arrived();

  /**
   * Checks if the Arrived method has finished and allows the next queued command
   * to run.
   * 
   * @return the state of the arrived method
   */
  public boolean isFinished() {
  if (arrivedCommands.empty() & runningCommand.isFinished()) {
      return true;
  } else {
    return false;
  }
  }

  /**
   * Ran if the bot cant get to the position it needs.
   * 
   * @param position The bots current position.
   */
  public abstract void fallback(Pose position);

  /**
   * <strong>DONT OVERRIDE.</strong> Override update instead. Overriding this will
   * cause the AutoTask to never start.
   */
  public void execute() {

  }

  /**
   * Use instead of execute. Functions as execute but with a position arguemnt.
   * 
   * @param position the current position of the robot when update is ran.
   */
  public abstract void update(Pose position);

  /**
   * Sets the position the bot will go to for the task.
   * <p>
   * Not running this in {@link #initTask initTask} <strong>will</strong> cause a
   * NullPointerException
   * 
   * @param position The position the bot will go to for the AutoTask
   */
  protected void setTaskPosition(Pose position) {
    taskPos = position;
  }

  /**
   * Ran if the task needs to be ended.
   */
  public void end() {
    if (!runningCommand.isFinished()) {
      runningCommand.end(true);
    }
  }

  /**
   * Adds command requirements for TaskScheduler (such as ending commands)
   * 
   * @param command The command/s to add
   */
  private void addCommandRequirement(CommandBase... command) {
    for (CommandBase cb : command) {
      commands.add(cb);
    }
  }

  /**
   * Sets the command to run on autotask init.
   * 
   * @param command The command to run.
   */
  protected void addInitCommand(CommandBase command) {
    addCommandRequirement(command);
    initCommands.push(command);
  }

  /**
   * Sets the command to run on autotask arrival.
   * 
   * @param command The command to run.
   */
  protected void addArrivedCommand(CommandBase command) {
    addCommandRequirement(command);
    arrivedCommands.push(command);
  }

  public void build() {
    if (taskPos == null) { // checks if taskpos was instantiated and if not throw an error
      throw new NullPointerException("taskPositon Was not ran in initTask.");
    }
  }
}
