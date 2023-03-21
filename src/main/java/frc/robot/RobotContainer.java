// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.ManualArm;
import frc.robot.commands.MoveToPose;
import frc.robot.commands.RetractArm;
import frc.robot.positioning.Pose;
import frc.robot.commands.DriveRobotOriented;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive driveSubsystem = new Drive();
  private Pose armPose;
  /* BIG CHUNGUS ARM CODE */
  private final ArmBase armBase = new ArmBase();
  private final ArmShoulder armShoulder = new ArmShoulder();
  private final ArmExtender armExtender = new ArmExtender();
  private final ArmClaw armClaw = new ArmClaw();
  private final CompilationArm compilationArm =
      new CompilationArm(armBase, armClaw, armExtender, armShoulder);
  private final ManualArm manualArm = new ManualArm(armBase, armShoulder, armExtender);
  private final Pose pose = new Pose();
  private RetractArm retractArmCommand = new RetractArm(armExtender);

  private final Balance balance = new Balance(driveSubsystem);
  private final DriveRobotOriented driveRO = new DriveRobotOriented(driveSubsystem);

  private final Command openClaw = armClaw.openClawCommand();

  private final Command extend = armExtender.ExtendCommand();
  private final Command retract = armExtender.RetractCommand();

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController controller =
      new CommandXboxController(OperatorConstants.CONTROLLER_NUMBER);

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_NUMBER);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public static double getRightXAxis() {

    return controller.getRightX();
  }

  public static double getRightYAxis() {
    return controller.getRightY();
  }

  public static double getLeftYAxis() {
    return controller.getLeftY();
  }

  private static double scaleAxis(double a) {
    return Math.signum(a) * Math.pow(a, 2);
  }

  public static double getScaledRightXAxis() {
    return scaleAxis(getRightXAxis());
  }

  public static double getScaledRightYAxis() {
    return scaleAxis(getRightYAxis());
  }

  public static double getScaledLeftYAxis() {
    return scaleAxis(getLeftYAxis());
  }

  

  public RobotContainer() {
    configureBindings();

    compilationArm.setDefaultCommand(manualArm);
  }

  

  public Pose containerGetArmPose() {

    return compilationArm.compGetArmPose();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
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

    controller.leftBumper().onTrue(extend);

    controller.rightBumper().onTrue(retract);

    joystick
        .button(2)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.RESET, armShoulder, armBase, armExtender, compilationArm));
    joystick
        .button(3)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.PICKUP, armShoulder, armBase, armExtender, compilationArm));
    joystick
        .button(11)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.SCORE_LOWER,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(7)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.SCORE_HIGH_CONE,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(9)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.SCORE_MID_CONE,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(8)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.SCORE_HIGH_CUBE,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(10)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.SCORE_MID_CUBE,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(12)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.HUMAN_SHELF,
                armShoulder,
                armBase,
                armExtender,
                compilationArm));
    joystick
        .button(1)
        .onTrue(
            new MoveToPose(
                Constants.Arm.Poses.CLOSE, armShoulder, armBase, armExtender, compilationArm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.homingRoutine(armShoulder, armBase);
  }

  

  public Command getResetCommand() {
    return new MoveToPose(
            Constants.Arm.Poses.RESET, armShoulder, armBase, armExtender, compilationArm)
        .alongWith(armClaw.openClawCommand());
  }

  public RetractArm getRetractCommand() {
    return retractArmCommand;
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
