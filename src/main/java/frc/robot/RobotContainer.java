// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController controller = new XboxController(OperatorConstants.CONTROLLER_NUMBER);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

    /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* Create JoystickButtons out of the controller IDs declared in constants */
    JoystickButton aButton = new JoystickButton(controller, Constants.ContollerButtons.A_NUMBER);
    JoystickButton bButton = new JoystickButton(controller, Constants.ContollerButtons.B_NUMBER);
    JoystickButton xButton = new JoystickButton(controller, Constants.ContollerButtons.X_NUMBER);
    JoystickButton yButton = new JoystickButton(controller, Constants.ContollerButtons.Y_NUMBER);
    JoystickButton leftBumper = new JoystickButton(controller, Constants.ContollerButtons.LEFT_BUMPER_NUMBER);
    JoystickButton rightBumper = new JoystickButton(controller, Constants.ContollerButtons.RIGHT_BUMPER_NUMBER);
    JoystickButton backButton = new JoystickButton(controller, Constants.ContollerButtons.BACK_NUMBER);
    JoystickButton startButton = new JoystickButton(controller, Constants.ContollerButtons.START_NUMBER);
    JoystickButton leftStick = new JoystickButton(controller, Constants.ContollerButtons.LEFT_STICK_NUMBER);
    JoystickButton rightStick = new JoystickButton(controller, Constants.ContollerButtons.RIGHT_STICK_NUMBER);
    POVButton dPadUp = new POVButton(controller, 0);
    POVButton dPadRight= new POVButton(controller, 90);        
    POVButton dPadDown = new POVButton(controller, 180);
    POVButton dPadLeft = new POVButton(controller, 270);

    new Trigger(dPadUp).onTrue(balance);
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
}
