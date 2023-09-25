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
