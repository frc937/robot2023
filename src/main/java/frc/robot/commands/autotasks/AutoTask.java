// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.commands.autotasks;

import java.util.ArrayList;
import java.util.Stack;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.Pose;
//TODO: Add fallback commands
/**
 * Base class for autotasks.
 * If you want to create an autotask extend this class.
 */
public abstract class AutoTask {
  private boolean initialized = false;
  private boolean arrived = false;
  private boolean ended = false;
  private boolean verified = false;
  private Pose taskPos;
  private ArrayList<CommandBase> commands = new ArrayList<CommandBase>();
  private Stack<CommandBase> initCommands = new Stack<CommandBase>();
  private Stack<CommandBase> arrivedCommands = new Stack<CommandBase>();
  private CommandBase runningCommand;
  /**
   * Creates a new AutoTask.
   * Dont create instances of commands.
   * Each command should be a parameter instead of created
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
  public boolean initFinished() {
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
   * Returns the acive command.
   */
  public CommandBase getActiveCommand() {
    return runningCommand;

  }

  /**
   * Like periodic but gives position arg
   * 
   * @param position The current position of the robot
   */
  public void updateTask(Pose position) {
    updateInit();
    updateArrived();
    update(position);
  }

  /*
   * Update the init sequence of the task. If the task is initialized the command
   * is bypassed.
   */
  private void updateInit() {
    /* Checks if the task has finished init sequence */
    if (!initialized) {
      /*
       * If task is not initalised, queue command if current running one is finished
       */
      if (runningCommand.isFinished() & !initCommands.isEmpty()) {
        runningCommand = initCommands.pop();
        /*
         * If currently running command is finished and there are no more init
         * Commands, initalize
         */
      } else if (initCommands.isEmpty() & runningCommand.isFinished()) {
        initialized = true;
      }
      /* Prevents current command from getting ran multiple times without intention */
      if (!runningCommand.isScheduled() & !runningCommand.isFinished()) {
        runningCommand.schedule();
      }
    }
  }

  /*
   * Update the arrived sequence of the task. If the task hasent arrived at the
   * desination the command is bypassed.
   */
  private void updateArrived() {
    /* Checks if the task has finished arrived sequence */
    if (!arrived) {
      /*
       * If task is not initalised, queue command if current running one is finished
       */
      if (runningCommand.isFinished() & !arrivedCommands.isEmpty()) {
        runningCommand = arrivedCommands.pop();
        /*
         * If currently running command is finished and there are no more init
         * Commands, end the command
         */
      } else if (arrivedCommands.isEmpty() & runningCommand.isFinished()) {
        ended = true;
      }
      /* Prevents current command from getting ran multiple times without intention */
      if (!runningCommand.isScheduled() & !runningCommand.isFinished()) {
        runningCommand.schedule();
      }
    }
  }

  /**
   * Returns if the task has ended or not.
   * 
   * @return The status of the command
   */
  public boolean ended() {
    return ended;
  }

  /**
   * Use instead of execute. Functions as execute but with a position arguemnt.
   * 
   * @param position the current position of the robot when update is ran.
   */
  protected abstract void update(Pose position);

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

  /**
   * Runs checks on the autotasks to make sure the tasks are valid
   */
  public void verify() {
    System.out.println();
    System.out.println("================================================================");
    System.out.println("Checking if taskPosition was ran in " + this.getClass().getName());
    if (taskPos == null) { // checks if taskpos was instantiated and if not throw an error
      System.out.println("Taskpos was not ran. In order for a AutoTask to be valid taskpos HAS to be ran in the constructor");
      throw new UnsupportedOperationException("Verification failed because taskpos was not ran in the constructor.");
      
    } else {
      verified = true;
      System.out.println("Taskpos was run. Task verified.");
    }
    System.out.println("================================================================");
  }
}
