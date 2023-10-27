// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.subsystems.Drive;

public class Balance extends CommandBase {

  private final Drive drive;
  private final DriveForwards forwards;
  private final DriveReverse reverse;

  /** Creates a new BalanceAuto */
  public Balance(Drive drive) {
    this.drive = drive;
    this.forwards = new DriveForwards(drive);
    this.reverse = new DriveReverse(drive);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchAngleDegrees = drive.getPitch(); // forward back
    
    if (pitchAngleDegrees > BalanceConstants.OFF_ANGLE_THRESHOLD) {
          this.forwards.execute();
    } else if (pitchAngleDegrees < -BalanceConstants.OFF_ANGLE_THRESHOLD) {
          this.reverse.execute();
    } else {
          this.reverse.end(true);
          this.forwards.end(true);
    }


    
    
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
