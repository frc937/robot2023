// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Stack;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.autotasks.AutoTask;

public class TaskScheduler extends SubsystemBase {
  /** Creates a new TaskController. */
  private Stack<AutoTask> taskqueue;
  private AutoTask currentTask;
  public TaskScheduler() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Adds a task to the end of the task queue.
   * @param task The autotask to queue
   */
  public void queueTask(AutoTask task){
    taskqueue.push(task);
  }
/**
 * Cancels the current task to run another one.
 * @param task The task to run instantly
 */
  public void runTaskNow(AutoTask task){
    currentTask.end(true);
    currentTask = task;
    initTask();
  }
/**
 * Removes a task from the task queue.<p>
 * If you are trying to skip the current running task, use {@link #skipTask()} instead.
 * @throws NullPointerException if task is not found in the queue.
 * @param task The task to remove from the taskqueue
 */
  public void removeTask(AutoTask task) throws NullPointerException {
    if (taskqueue.contains(task)){
      taskqueue.remove(task);
    } else {
      throw new NullPointerException("Cant find task in queue");
    }
  }
/**
 * Skips the currently scheduled class.
 */
  public void skipTask(){
    stopTask();
    nextTask();
  }
/**
 * Clears the task queue. If there are any tasks queued they will be removed. <p>
 * Doesnt stop the current task (if one is running).
 */
  public void clearQueue(){
    taskqueue.clear();
  }
/**
 * Gets the next queued task.
 */
  private void nextTask(){
    if (!taskqueue.isEmpty()){
    currentTask = taskqueue.pop();
    initTask();
  }}
/**
 * Stops the current running task. Will not pop the next command.
 */
  private void stopTask(){
    currentTask.end(true);
  }
/**
 * Inits a command.
 */
  private void initTask(){
    currentTask.initTask();
  }

}
