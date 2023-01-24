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

  public void queueTask(AutoTask task){
    taskqueue.push(task);
  }

  public void runTaskNow(AutoTask task){
    currentTask.cancel();
    currentTask = task;
    initTask();
  }

  public void removeTask(AutoTask task){
    if (taskqueue.contains(task)){
      taskqueue.remove(task);
    }
  }

  public void skipTask(){
    currentTask.cancel();
    nextTask();
  }
  
  public void clearQueue(){
    taskqueue.clear();
  }

  private void nextTask(){
    if (!taskqueue.isEmpty()){
    currentTask = taskqueue.pop();
    initTask();
  }}

  private void stopTask(){
    currentTask.cancel();
  }

  private void initTask(){
    currentTask.initialize();
  }

}
