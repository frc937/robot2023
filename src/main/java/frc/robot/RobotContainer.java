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
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  
  /* BIG CHUNGUS ARM CODE */
  private final ArmBase armBase = new ArmBase();
  private final ArmShoulder armShoulder = new ArmShoulder();
  private final ArmExtender armExtender = new ArmExtender();
  private final ArmClaw armClaw = new ArmClaw();
  private final CompilationArm compilationArm = new CompilationArm(armBase, armClaw, armExtender, armShoulder);

  private final Balance balance = new Balance(driveSubsystem);

  private final Command openClaw = armClaw.openClawCommand();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController controller = new CommandXboxController(OperatorConstants.CONTROLLER_NUMBER);

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_NUMBER); 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public static double getRightXAxis () {

    return controller.getRightX();
  }

  public static double getRightYAxis() {
    return controller.getRightY();
  }

  private static double scaleAxis(double a) {
    return math.signum(a) * Math.pow(a, 2);
  }

  public static double getScaledRightXAxis() {
    return scaleAxis(getRightXAxis());
  }

  public static double getScaledRightYAxis() {
    return scaleAxis(getRightYAxis());
  }

  public RobotContainer() {
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

    /* this is super not the way we do this anymore */
    /*JoystickButton aButton = new JoystickButton(controller, Constants.ContollerButtons.A_NUMBER);
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
    POVButton dPadLeft = new POVButton(controller, 270);*/

    joystick.trigger().onTrue(openClaw);

    controller.povUp().onTrue(balance);

    joystick.button(2).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.RESET));
    joystick.button(3).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.PICKUP));
    joystick.button(11).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.SCORE_LOWER));
    joystick.button(7).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.SCORE_HIGH_CONE));
    joystick.button(9).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.SCORE_MID_CONE));
    joystick.button(8).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.SCORE_HIGH_CUBE));
    joystick.button(10).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.SCORE_MID_CUBE));
    joystick.button(12).onTrue(compilationArm.moveToPoseCommand(Constants.Arm.Poses.HUMAN_SHELF));
    
    

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
