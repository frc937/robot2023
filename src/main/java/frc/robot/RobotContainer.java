// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveRobotOriented;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive driveSubsystem = new Drive();

  private final Balance balance = new Balance(driveSubsystem);
  private final DriveRobotOriented driveRO = new DriveRobotOriented(driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final static CommandXboxController controller = new CommandXboxController(
      OperatorConstants.CONTROLLER_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driveSubsystem.setDefaultCommand(driveRO);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.povUp().onTrue(balance);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  private static double scaleAxis(double a) {
    return Math.signum(a) * Math.pow(a, 2);
  }

  public static double getLeftXAxis() {
    return controller.getLeftX();
  }

  public static double getScaledLeftXAxis() {
    return scaleAxis(getLeftXAxis());
  }

  public static double getLeftYAxis() {
    return controller.getLeftY() * -1.0;
  }

  public static double getScaledLeftYAxis() {
    return scaleAxis(getLeftYAxis());
  }

  public static double getRightXAxis() {
    return controller.getRightX();
  }

  public static double getScaledRightXAxis() {
    return scaleAxis(getRightXAxis());
  }

  public static double getRightYAxis() {
    return controller.getRightY() * -1.0;
  }

  public static double getScaledRightYAxis() {
    return scaleAxis(getRightYAxis());
  }

}
