// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.Drive;

public class Balance extends CommandBase {
  private boolean BalanceXMode;
  private boolean BalanceYMode;

  private final Drive drive;

  /** Creates a new BalanceAuto */
  public Balance(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = drive.getPitch();
    double rollAngleDegrees = drive.getRoll();
    double xAxisRate = 0;
    double yAxisRate = 0;
    if (!BalanceXMode
        && (Math.abs(pitchAngleDegrees) >= Math.abs(BalanceConstants.OFF_ANGLE_THRESHOLD))) {
      BalanceXMode = true;
    } else if (BalanceXMode
        && (Math.abs(pitchAngleDegrees) <= Math.abs(BalanceConstants.ON_ANGLE_THRESHOLD))) {
      BalanceXMode = false;
    }

    if (!BalanceYMode
        && (Math.abs(pitchAngleDegrees) >= Math.abs(BalanceConstants.OFF_ANGLE_THRESHOLD))) {
      BalanceYMode = true;
    } else if (BalanceYMode
        && (Math.abs(pitchAngleDegrees) <= Math.abs(BalanceConstants.ON_ANGLE_THRESHOLD))) {
      BalanceYMode = false;
    }
    if (BalanceXMode) {
      double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
      xAxisRate = Math.sin(pitchAngleRadians) * -1;
    }
    if (BalanceYMode) {
      double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
      yAxisRate = Math.sin(rollAngleRadians) * -1;
    }
    drive.moveMecanumRobot(yAxisRate, xAxisRate, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
