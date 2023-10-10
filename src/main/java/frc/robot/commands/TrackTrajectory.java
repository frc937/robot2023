// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

/**
 * Command to track a trajectory. You'll need to construct a new one for each different trajectory
 * you want to track.
 */
public class TrackTrajectory extends CommandBase {
  Drive drive;
  Trajectory trajectory;
  Rotation2d desiredRotation;
  double FPGAOffset;

  /**
   * Constructs a TrackTrajectory command. Will need to be run separately for each trajectory you
   * want to track.
   *
   * @param trajectory The trajectory to track.
   * @param drive Drive subsysem for dependency injection.
   */
  public TrackTrajectory(Trajectory trajectory, Drive drive) {
    this.trajectory = trajectory;
    this.drive = drive;
    addRequirements(drive);
  }

  /**
   * Called by the scheduler when the command is initially scheduled. Sets the offset for the timer
   * and begins tracking the trajectory.
   */
  @Override
  public void initialize() {
    FPGAOffset = Timer.getFPGATimestamp();
    drive.trackTrajectory(trajectory.sample(0));
  }

  /** Called each scheduler run while the command is scheduled. Tracks the trajectory. */
  @Override
  public void execute() {
    drive.trackTrajectory(trajectory.sample(Timer.getFPGATimestamp() - FPGAOffset));
  }

  /** Called once the command ends or is interrupted. Not used in this class. */
  @Override
  public void end(boolean interrupted) {}

  /**
   * Returns true when the command should end. Will return true when {@link Drive}'s
   * holonomicController is done tracking the trajectory.
   */
  @Override
  public boolean isFinished() {
    return drive.trajectoryDone();
  }
}
