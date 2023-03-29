// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Plunger;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DeployPlunger;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.commands.ManualArm;
import frc.robot.commands.RetractArm;
import frc.robot.commands.StartLeavingCommunity;
import frc.robot.commands.StopLeavingCommunity;
import frc.robot.commands.moveToPose.MoveToPose;
import frc.robot.positioning.Pose;
import frc.robot.commands.DriveRobotOriented;
import frc.robot.commands.ExtendArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.I2CManager;
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
  private final Drive driveSubsystem = new Drive();
  /* BIG CHUNGUS ARM CODE */
  private final I2CManager I2CManager = new I2CManager();
  private final ArmBase armBase = new ArmBase();
  private final ArmShoulder armShoulder = new ArmShoulder();
  private final ArmExtender armExtender = new ArmExtender(I2CManager);
  private final ArmClaw armClaw = new ArmClaw();
  private final CompilationArm compilationArm =
      new CompilationArm(armBase, armClaw, armExtender, armShoulder);
  private final ManualArm manualArm = new ManualArm(armBase, armShoulder, compilationArm);
  private RetractArm retractArmCommand = new RetractArm(armExtender);
  private final Plunger plunger = new Plunger();
  private final DeployPlunger deployPlunger = new DeployPlunger(plunger);
  private final StartLeavingCommunity startLeavingCommunity = new StartLeavingCommunity(driveSubsystem);
  private final StopLeavingCommunity stopLeavingCommunity = new StopLeavingCommunity(driveSubsystem);

  private final Balance balance = new Balance(driveSubsystem);
  private final DriveRobotOriented driveRO = new DriveRobotOriented(driveSubsystem);
  private final DriveFieldOriented driveFO = new DriveFieldOriented(driveSubsystem);

  private final Command openClaw = armClaw.manualOpenClawCommand();
  private final Command closeClaw = armClaw.manualCloseClawCommand();

  private final ExtendArm extend = new ExtendArm(armExtender);
  private final RetractArm retract = new RetractArm(armExtender);

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController controller =
      new CommandXboxController(OperatorConstants.CONTROLLER_NUMBER);

  public static CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_NUMBER);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */


  

  public RobotContainer() {
    configureBindings();

    compilationArm.setDefaultCommand(manualArm);
    driveSubsystem.setDefaultCommand(driveRO);
  }

  

  public Pose containerGetArmPose() {

    return compilationArm.getArmPose();
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

    //controller.y().whileTrue(deployPlunger);

    controller.rightBumper().whileTrue(balance);

    controller.y().toggleOnTrue(driveFO);

    controller.b().whileTrue(deployPlunger);

    //controller.x().whileTrue(MoveToPose.extendingMoveToPose(Constants.Arm.Poses.PICKUP, armBase, armShoulder, armExtender, compilationArm));

    joystick.povUp().whileTrue(extend);
    joystick.povDown().whileTrue(retract);

    joystick.button(11).whileTrue(openClaw);
    joystick.trigger().whileTrue(closeClaw);

    /*joystick
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
        .onTrue(armClaw.manualCloseClawCommand());*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return (Autos.homingRoutine(armShoulder, armBase,armExtender,armClaw,compilationArm)).andThen(startLeavingCommunity).andThen(new WaitCommand(2)).andThen(stopLeavingCommunity);
  }

  

  public Command getResetCommand() {
    return MoveToPose.retractingMoveToPose(Constants.Arm.Poses.RESET, armBase, armShoulder, armExtender, compilationArm)
        .alongWith(armClaw.openClawCommand());
  }

  public RetractArm getRetractCommand() {
    return retractArmCommand;
  }

  private static double scaleAxis(double a) {
    return Math.signum(a) * Math.pow(a, 2);
  }

  public static double getControllerLeftXAxis() {
    return controller.getLeftX();
  }

  public static double getScaledControllerLeftXAxis() {
    return scaleAxis(getControllerLeftXAxis());
  }

  public static double getControllerLeftYAxis() {
    return controller.getLeftY() * -1.0;
  }

  public static double getScaledControllerLeftYAxis() {
    return scaleAxis(getControllerLeftYAxis());
  }

  public static double getControllerRightXAxis() {
    return controller.getRightX();
  }

  public static double getScaledControllerRightXAxis() {
    return scaleAxis(getControllerRightXAxis());
  }

  public static double getControllerRightYAxis() {
    return controller.getRightY() * -1.0;
  }

  public static double getScaledControllerRightYAxis() {
    return scaleAxis(getControllerRightYAxis());
  }

  public static double getJoystickXAxis() {
    return joystick.getX();
  }

  public static double getScaledJoystickXAxis() {
    return scaleAxis(getJoystickXAxis());
  }

  public static double getJoystickYAxis() {
    return joystick.getY() * -1.0;
  }

  public static double getScaledJoystickYAxis() {
    return scaleAxis(getJoystickYAxis());
  }

}
