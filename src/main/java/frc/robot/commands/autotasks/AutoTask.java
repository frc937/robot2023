// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. 

package frc.robot.commands.autotasks;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.positioning.Pose;
import frc.robot.positioning.Path;
import frc.robot.Constants;
import frc.robot.positioning.AStar;
import frc.robot.subsystems.Drive;
import java.util.concurrent.atomic.AtomicReference;
// TODO: Add fallback commands

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
  private CommandBase runningCommand;
  private CommandBase initCommand;
  private CommandBase arrivedCommand;
  private State commandState;
  private AStar aStar;
  private AtomicReference<Path> path;

  private enum State {
    INIT,
    NAVIGATING,
    ARRIVED,
    FINISHED
    
  }
  /**
   * Creates a new AutoTask. Dont create instances of commands. Each command should be a parameter
   * instead of created
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
    if (commandState.ordinal() > 1) {
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
    if (commandState == State.FINISHED) {
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
    if (commandState == State.INIT) {
      if (!runningCommand.equals(initCommand)) {
        runningCommand = initCommand;
      }
      if (!runningCommand.isScheduled() & !runningCommand.isFinished()) {
        runningCommand.schedule();
      } else if (runningCommand.isFinished()){
        commandState = State.NAVIGATING;
      }
    }
  }

  /*
   * Update the arrived sequence of the task. If the task hasent arrived at the
   * desination the command is bypassed.
   */
  private void updateArrived() {
    /* Checks if the task has finished arrived sequence */
    if (commandState == State.ARRIVED) {
      if (!runningCommand.equals(arrivedCommand)) {
        runningCommand = arrivedCommand;
      }
      if (!runningCommand.isScheduled() & !runningCommand.isFinished()) {
        runningCommand.schedule();
      } else if (runningCommand.isFinished()){
        commandState = State.FINISHED;
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
    //TODO: THESE VALUES ARE VERY MUCH PLACEHOLERS - LIKE RIGHT HERE V
    aStar = new AStar(
      Constants.RobotDimensions.CENTER_POSE.getWorldOriented(new Pose()).getX(), 
      Constants.RobotDimensions.CENTER_POSE.getWorldOriented(new Pose()).getY(), 
      position.getX(), 
      position.getY()
      );
  }

  /**
   * Generates the task path (AStar path)
   * <p> Make sure to use isPathGenerated() otherwise you will be grabbing a null path.
   */
  protected void generateTaskPath() {
    path = aStar.generateAStarPath();
  }

  /**
   * Chekcs whether or not the path has been generated.
   * Make sure the path has been generated before utilizing the path.
   * @return
   */
  protected boolean isPathGenerated() {
    return path.get().isPathGenerated();
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
   * Sets the command to run on autotask init.
   * 
   * @param command The command to run.
   */
  protected void setInitCommand(CommandBase command) {
    initCommand = command;
  }

  /**
   * Sets the command to run on autotask arrival.
   * 
   * @param command The command to run.
   */
  protected void setArrivedCommand(CommandBase command) {
    arrivedCommand = command;
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
