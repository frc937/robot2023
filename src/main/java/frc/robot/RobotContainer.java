// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Asimov's Laws:
 * The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
 * The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
 * The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.
 */
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.DriveReverse;
import frc.robot.commands.DriveTank;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.InCubeOutCone;
import frc.robot.commands.ManualArm;
import frc.robot.commands.OutCubeInCone;
import frc.robot.commands.ResetDrivePose;
import frc.robot.commands.RetractArm;
import frc.robot.commands.StartLeavingCommunity;
import frc.robot.commands.StopLeavingCommunity;
import frc.robot.commands.TrackTrajectory;
import frc.robot.commands.moveToPose.MoveToPose;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.I2CManager;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightManager;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmIntake;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  /* ===============================================================
   * SUBSYSTEMS
   * ===============================================================
   */

  /* LIMELIGHT  */
  private final Limelight limelightBack = new Limelight("limelight-back");
  private final Limelight limelightFront = new Limelight("limelight-front");
  private final LimelightManager limelightManager =
      new LimelightManager(limelightBack, limelightFront);
  /* CAMERA */
  private final Camera aimCamera = new Camera(Constants.Camera.PORT_CAMERA_AIM);
  /* DRIVE */
  private final Drive driveSubsystem = new Drive(limelightManager);
  /* ARM */
  private final I2CManager I2CManager = new I2CManager();
  private final ArmBase armBase = new ArmBase();
  private final ArmShoulder armShoulder = new ArmShoulder();
  private final ArmExtender armExtender = new ArmExtender(I2CManager);
  private final ArmIntake armClaw = new ArmIntake();
  private final CompilationArm compilationArm =
      new CompilationArm(armBase, armClaw, armExtender, armShoulder);
  private final ManualArm manualArm = new ManualArm(armBase, armShoulder, compilationArm);
  private RetractArm retractArmCommand = new RetractArm(armExtender);

  /* ===============================================================
   * COMMANDS
   * ===============================================================
   */

  /* AUTO COMMANDS */
  private final StartLeavingCommunity startLeavingCommunity =
      new StartLeavingCommunity(driveSubsystem);
  private final StopLeavingCommunity stopLeavingCommunity =
      new StopLeavingCommunity(driveSubsystem);
  /* CAMERA COMMAND */
  private final InstantCommand displayAimVideo =
      new InstantCommand(aimCamera::startCamera, aimCamera);
  /* BALANCE COMMAND */
  private final Balance balance = new Balance(driveSubsystem);
  /* DRIVE COMMANDS*/
  private final DriveArcade driveRO = new DriveArcade(driveSubsystem);
  private final DriveTank driveFO = new DriveTank(driveSubsystem);
  private final DriveForwards driveForwards = new DriveForwards(driveSubsystem);
  private final DriveReverse driveReverse = new DriveReverse(driveSubsystem);
  /* ARM COMMANDS */
  private final InCubeOutCone inCubeOutCone = new InCubeOutCone(armClaw);
  private final OutCubeInCone outCubeInCone = new OutCubeInCone(armClaw);
  private final ExtendArm extend = new ExtendArm(armExtender);
  private final RetractArm retract = new RetractArm(armExtender);
  /* TRAJECTORY COMMAND */
  private final TrackTrajectory demoTrajectoryTrackingCommand =
      new TrackTrajectory(Constants.Drive.Trajectories.DEMO_TRAJECTORY, driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController controller =
      new CommandXboxController(OperatorConstants.CONTROLLER_NUMBER);

  public static CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_NUMBER);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    /* TODO: add ResetDrivePose into auto*/
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption(
        "Fling cube + mobility bonus",
        Commands.sequence(
            Autos.homingRoutine(armShoulder, armBase, armExtender, armClaw, compilationArm),
            new ParallelRaceGroup(new DriveForwards(driveSubsystem), new WaitCommand(0.5)),
            new ParallelRaceGroup(new DriveReverse(driveSubsystem), new WaitCommand(0.8)),
            new ParallelRaceGroup(new DriveForwards(driveSubsystem), new WaitCommand(2.5))));
    autoChooser.addOption(
        "Home arm (DOES NOT MOVE)",
        Autos.homingRoutine(armShoulder, armBase, armExtender, armClaw, compilationArm));

    SmartDashboard.putData("Choose auto", autoChooser);

    compilationArm.setDefaultCommand(manualArm);
    driveSubsystem.setDefaultCommand(driveFO);
  }

  public Pose containerGetArmPose() {
    return compilationArm.getArmPose();
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

    controller.rightBumper().whileTrue(balance);

    controller.leftBumper().whileTrue(new InstantCommand(driveSubsystem::printPoses));

    controller
        .a()
        .whileTrue(
            new TrackTrajectory(
                Constants.Trajectories.LOADING_ZONE_TO_UPPER_COMMUNITY, driveSubsystem));

    controller
        .b()
        .whileTrue(
            new TrackTrajectory(
                Constants.Trajectories.LOADING_ZONE_TO_LOWER_COMMUNITY, driveSubsystem));

    controller
        .x()
        .whileTrue(
            new TrackTrajectory(
                Constants.Trajectories.UPPER_COMMUNITY_TO_LOADING_ZONE, driveSubsystem));

    controller
        .y()
        .whileTrue(
            new TrackTrajectory(
                Constants.Trajectories.LOWER_COMMUNITY_TO_LOADING_ZONE, driveSubsystem));

    controller.x().whileTrue(demoTrajectoryTrackingCommand);

    joystick
        .button(9)
        .whileTrue(
            MoveToPose.extendingMoveToPose(
                Constants.Arm.Poses.PICKUP, armBase, armShoulder, armExtender, compilationArm));
    joystick
        .button(4)
        .whileTrue(Autos.homingNoOpenClaw(armShoulder, armBase, armExtender, compilationArm));
    joystick.povUp().whileTrue(extend);
    joystick.povDown().whileTrue(retract);
    // in cube out cube
    joystick.button(3).whileTrue(inCubeOutCone);
    joystick.button(4).whileTrue(outCubeInCone);
    // in cone out cone
    joystick.button(5).whileTrue(outCubeInCone);
    joystick.button(6).whileTrue(inCubeOutCone);
    // joystick.button(2).whileTrue(closeClawCube);

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
        .onTrue(armClaw.manualCloseClawCommand());
        */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public Command getResetCommand() {
    return MoveToPose.retractingMoveToPose(
        Constants.Arm.Poses.RESET, armBase, armShoulder, armExtender, compilationArm);
  }

  public Command getRetractCommand() {
    return retractArmCommand;
  }

  public Command getDisplayAimVideoCommand() {
    return displayAimVideo;
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
    return joystick.getX() * -1.0;
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

  public ResetDrivePose getResetDrivePoseCommand() {
    return new ResetDrivePose(limelightManager, driveSubsystem);
  }
}
