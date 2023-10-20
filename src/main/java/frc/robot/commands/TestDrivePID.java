// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.Constants;

public class TestDrivePID extends CommandBase {
  private Drive drive;

  /** Creates a new TestDrivePID. */
  public TestDrivePID(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setVelocity(Constants.Drive.TEST_PID_SETPOINT, Constants.Drive.TEST_PID_SETPOINT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
