package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class DriveTank extends CommandBase {

  /* Variables */
  private final Drive drivetrain;
  private double left;
  private double right;

  public DriveTank(Drive driveSubsystem) {
    drivetrain = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString("Drive Perspective", "Field");
  }

  @Override
  public void execute() {
    /* Gets the left and right axes of the robot and uses that to move */
    left = RobotContainer.getScaledControllerLeftYAxis();
    right = RobotContainer.getScaledControllerRightYAxis();
    drivetrain.moveTank(left, right);
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
