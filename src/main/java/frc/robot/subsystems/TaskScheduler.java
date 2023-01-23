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
  
  public void QueueTask(AutoTask task){
    taskqueue.push(task);
  }

  public void RunTaskNow(AutoTask task){
    currentTask.cancel();
    currentTask = task;
    InitTask();
  }

  public void RemoveTask(AutoTask task){
    if (taskqueue.contains(task)){
      taskqueue.remove(task);
    }
  }

  private void NextTask(){
    if (!taskqueue.isEmpty()){
    currentTask = taskqueue.pop();
    InitTask();
  }}

  private void InitTask(){ // probaly will need more than just initalize
    currentTask.initialize();
  }

}
