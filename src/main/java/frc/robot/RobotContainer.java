// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.TaskScheduler;
import frc.robot.commands.CloseClawCone;
import frc.robot.commands.CloseClawCube;
import frc.robot.commands.DeployPlunger;
import frc.robot.commands.DriveTank;
import frc.robot.commands.DriveForwards;
import frc.robot.commands.DriveReverse;
import frc.robot.commands.DriveArcade;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.ManualArm;
import frc.robot.commands.RetractArm;
import frc.robot.commands.StartLeavingCommunity;
import frc.robot.commands.StopLeavingCommunity;
import frc.robot.commands.TrackTrajectory;
import frc.robot.commands.moveToPose.MoveToPose;
import frc.robot.positioning.Pose;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.I2CManager;
import frc.robot.subsystems.Plunger;
import frc.robot.subsystems.arm.ArmBase;
import frc.robot.subsystems.arm.ArmClaw;
import frc.robot.subsystems.arm.ArmExtender;
import frc.robot.subsystems.arm.ArmShoulder;
import frc.robot.subsystems.arm.CompilationArm;

@SuppressWarnings("unused")
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  /* Messy, ugly commands and subsystems section
   * TODO: organize later
   */
  /* Autotasks are mostly commented out in here for now because I don't care that they exist */
  private final Limelight limelight = new Limelight();
  private final Drive driveSubsystem = new Drive(limelight);
  /* BIG CHUNGUS ARM CODE */
  //private final I2CManager I2CManager = new I2CManager();
  //private final ArmBase armBase = new ArmBase();
  //private final ArmShoulder armShoulder = new ArmShoulder();
  //private final ArmExtender armExtender = new ArmExtender(I2CManager);
  //private final ArmClaw armClaw = new ArmClaw();
  private final Camera aimCamera = new Camera(Constants.Camera.PORT_CAMERA_AIM);
  //private final CompilationArm compilationArm =
  //    new CompilationArm(armBase, armClaw, armExtender, armShoulder);
  //private final ManualArm manualArm = new ManualArm(armBase, armShoulder, compilationArm);
  //private RetractArm retractArmCommand = new RetractArm(armExtender);
  private final Plunger plunger = new Plunger();
  private final DeployPlunger deployPlunger = new DeployPlunger(plunger);
  private final StartLeavingCommunity startLeavingCommunity = new StartLeavingCommunity(driveSubsystem);
  private final StopLeavingCommunity stopLeavingCommunity = new StopLeavingCommunity(driveSubsystem);
  private final DriveForwards driveForwards = new DriveForwards(driveSubsystem);
  private final DriveReverse driveReverse = new DriveReverse(driveSubsystem);
  private final InstantCommand displayAimVideo = new InstantCommand(aimCamera::startCamera, aimCamera);
  private final Balance balance = new Balance(driveSubsystem);
  private final DriveArcade driveRO = new DriveArcade(driveSubsystem);
  private final DriveTank driveFO = new DriveTank(driveSubsystem);
  //private final Command openClaw = armClaw.manualOpenClawCommand();
  //private final Command closeClaw = armClaw.manualCloseClawCommand();
  //private final CloseClawCone closeClawCone = new CloseClawCone(armClaw);
  //private final CloseClawCube closeClawCube = new CloseClawCube(armClaw);
  //private final ExtendArm extend = new ExtendArm(armExtender);
  //private final RetractArm retract = new RetractArm(armExtender);
  //private final TaskScheduler taskScheduler = new TaskScheduler();
  private final TrackTrajectory demoTrajectoryTrackingCommand = new TrackTrajectory(Constants.Drive.Trajectories.DEMO_TRAJECTORY, driveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandXboxController controller =
      new CommandXboxController(OperatorConstants.CONTROLLER_NUMBER);

  public static CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_NUMBER);

  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    autoChooser = new SendableChooser<>();
    //autoChooser.setDefaultOption("Fling cube + mobility bonus", 
    //    Commands.sequence(Autos.homingRoutine(armShoulder, armBase, armExtender, armClaw, compilationArm), new ParallelRaceGroup(new DriveForwards(driveSubsystem), new WaitCommand(0.5)), new ParallelRaceGroup(new DriveReverse(driveSubsystem), new WaitCommand(0.8)), new ParallelRaceGroup(new DriveForwards(driveSubsystem), new WaitCommand(2.5))));
    //autoChooser.addOption("Home arm (DOES NOT MOVE)", Autos.homingRoutine(armShoulder, armBase, armExtender, armClaw, compilationArm));

    SmartDashboard.putData("Choose auto", autoChooser);

    //compilationArm.setDefaultCommand(manualArm);
    driveSubsystem.setDefaultCommand(driveRO);
  }

  public Pose containerGetArmPose() {

    //return compilationArm.getArmPose();
    return new Pose();
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

    // controller.y().whileTrue(deployPlunger);

    controller.rightBumper().whileTrue(balance);

    controller.y().toggleOnTrue(driveFO);

    controller.b().whileTrue(deployPlunger);

    controller.x().whileTrue(demoTrajectoryTrackingCommand);

    //joystick.button(9).whileTrue(MoveToPose.extendingMoveToPose(Constants.Arm.Poses.PICKUP, armBase,
    //armShoulder, armExtender, compilationArm));

    //joystick.button(4).whileTrue(Autos.homingNoOpenClaw(armShoulder, armBase, armExtender, compilationArm));

    //joystick.povUp().whileTrue(extend);
    //joystick.povDown().whileTrue(retract);

    //joystick.button(11).whileTrue(openClaw);
    /* TODO: get better buttons with Gabriel */
    /* Or just get color sensor working */
    //joystick.trigger().whileTrue(closeClaw);
    //joystick.button(2).whileTrue(closeClawCube);

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

  private void verifyAutoTasks() {
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
    //return MoveToPose.retractingMoveToPose(
    //        Constants.Arm.Poses.RESET, armBase, armShoulder, armExtender, compilationArm)
    //    .alongWith(armClaw.openClawCommand());
    return new InstantCommand();
  }

  public Command getRetractCommand() {
    //return retractArmCommand;
    return new InstantCommand();
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
    return joystick.getX()* -1.0;
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
