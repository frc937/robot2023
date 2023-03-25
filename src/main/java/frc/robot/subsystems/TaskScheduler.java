// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TrackTrajectory;
import frc.robot.commands.autotasks.AutoTask;
import frc.robot.commands.autotasks.AutoTask.State;
import frc.robot.positioning.AStarTrajectoryGenerator;
import frc.robot.positioning.Path;
import frc.robot.positioning.Pose;
import java.util.Stack;

import javax.sound.midi.Track;

public class TaskScheduler extends SubsystemBase {
  /** Creates a new TaskController. */
  private Stack<AutoTask> taskqueue;

  private AutoTask currentTask;
  private Pose botPosition;
  private Drive drive;
  private boolean isPathGenerated = false;
  private boolean hasArrived = false;
  private Trajectory currPath;
  private TrackTrajectory trajectory;

  public TaskScheduler(Drive drive) {
      this.drive = drive;
  }

  @Override
  public void periodic() {
    if (currentTask != null) {
      if (currentTask.getPathReferece().get().isPathGenerated() & !isPathGenerated) {
        currPath = AStarTrajectoryGenerator.generateTrajectory(currentTask.getPathReferece().get());
        if (currentTask.isInitFinished()) {
          currentTask.setTaskState(State.NAVIGATING);
          isPathGenerated = true;
          this.trajectory = new TrackTrajectory(currPath, currentTask.getPose().toPose2d().getRotation(), drive);
          this.trajectory.schedule();
        }
      } else if (isPathGenerated) {
        if (trajectory.isFinished() & !hasArrived) {
          this.currentTask.setTaskState(State.ARRIVED);
          hasArrived = true;
        }
      }
    if (!currentTask.ended()) {
      currentTask.updateTask(botPosition);
    } else {
      nextTask();
    }
    this.botPosition = new Pose(drive.getMecanumDrivePoseEstimator().getEstimatedPosition());
  }
  }

  /**
   * Adds a task to the end of the task queue.
   *
   * @param task The autotask to queue
   */
  public void queueTask(AutoTask task) {
    taskqueue.push(task);
  }

  /**
   * Cancels the current task to run another one.
   *
   * @param task The task to run instantly
   */
  public void runTaskNow(AutoTask task) {
    stopTask();
    currentTask = task;
    initTask();
  }

  /**
   * Removes a task from the task queue.
   *
   * <p>If you are trying to skip the current running task, use {@link #skipTask()} instead.
   *
   * @throws NullPointerException if task is not found in the queue.
   * @param task The task to remove from the taskqueue
   */
  public void removeTask(AutoTask task) throws NullPointerException {
    if (taskqueue.contains(task)) {
      taskqueue.remove(task);
    } else {
      throw new NullPointerException("Cant find task in queue");
    }
  }

  /** 
  * Skips the current AutoTask and queues the next one if it exists.
  */
  public void skipTask() {
    stopTask();
    nextTask();
  }

  /**
   * Clears the task queue. If there are any tasks queued they will be removed.
   *
   * <p>Doesnt stop the current task (if one is running).
   */
  public void clearQueue() {
    taskqueue.clear();
  }

  /** 
   *  Executes the next task in the queue if it isnt empty. If it is empty this method does nothing.
  */
  private void nextTask() {
    if (!taskqueue.isEmpty()) {
      currentTask = taskqueue.pop();
      initTask();
    }
  }

  /** Stops the current running task. 
   *  This will not pop the next command. (This wont have the next one run in the queue run.) 
   * */
  private void stopTask() {
    currentTask.end();
  }

  /**
   * Initalizes an AutoTask. <p>
   * Used when an AutoTask is ran or when the next one in queue is ran
   */
  private void initTask() {
    currentTask.initTask();
    currentTask.generateTaskPath();
  }

  /**
   * Sets the current bot position.
   *
   * @param position
   */
  public void setBotPosition(Pose position) {
    this.botPosition = position;
  }
}
