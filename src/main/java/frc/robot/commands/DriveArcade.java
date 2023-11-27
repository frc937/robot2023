// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Asimov's Laws:
 * The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
 * The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
 * The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveArcade extends CommandBase {

  /* Variables */
  private final Drive drivetrain;
  private double x;
  private double z;

  public DriveArcade(Drive driveSubsystem) {
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Perspective", "Robot");
  }

  @Override
  public void execute() {
    /* Gets the left and right axes of the robot and uses that to move */
    x = RobotContainer.getScaledControllerLeftYAxis();
    z = RobotContainer.getScaledControllerRightXAxis();
    /*mecanumZ = RobotContainer.getScaledRightXAxis();*/
    drivetrain.moveArcade(x, z);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
