// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveRobotOriented extends CommandBase {

  /* Variables */
  private final Drive drivetrain;
  private double mecanumX;
  private double mecanumY;
  private double mecanumZ;

  public DriveRobotOriented(Drive driveSubsystem) {
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
    mecanumX = RobotContainer.getScaledControllerLeftXAxis();
    mecanumY = RobotContainer.getScaledControllerLeftYAxis();
    mecanumZ = RobotContainer.getScaledControllerRightXAxis();
    /*mecanumZ = RobotContainer.getScaledRightXAxis();*/
    drivetrain.moveMecanumRobot(mecanumY, mecanumX, mecanumZ);
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
