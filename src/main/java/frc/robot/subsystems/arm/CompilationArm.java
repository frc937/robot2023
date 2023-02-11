// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.positioning.ArmKinematics;
import frc.robot.positioning.Pose;

public class CompilationArm extends SubsystemBase {
  private ArmBase armBase;
  private ArmClaw armClaw;
  private ArmExtender armExtender;
  private Pose armPose;
  private ArmShoulder armShoulder;


  /** Creates a new CompilationArm. */


  public CompilationArm(ArmBase armBase, ArmClaw armClaw, ArmExtender armExtender, ArmShoulder armShoulder) {
    this.armBase = armBase;
    this.armClaw = armClaw;
    this.armExtender = armExtender;
    this.armShoulder = armShoulder;

    updateArmPose();
  }

  /**
   * Corrects the different basis between the sensor and the math model

   */
  private double correctShoulderAngle(final double angle) {
    // In the model, cos(UP) = 1
    // and sin(FORWARD) = 1
    return angle;
  }

  /**
   * Returns whether or not the arm is in danger of overextending.
   */
  public boolean isAlmostOverextended() {
    return ArmKinematics.isAlmostOverextended(armPose);
  }

  /**
   * Returns whether or not the arm is overextended. If this is the case, the
   * robot MUST immediately retract the arm to avoid penalties.
   */
  public boolean isOverextended() {
    return ArmKinematics.isOverextended(armPose);
  }

  /**
   * Returns whether the robot is stabbing itself. Keep in mind, this only tests
   * whether or not the end effector is stabbing the frame. If anything is built
   * up above KEEP_OUT_HEIGHT, this will happily run into it.
   */
  private boolean isStabbingSelf() {
    return ArmKinematics.isStabbingSelf(armPose);
  }

  /**
   * getter method of the position of the arm 
   * @return Current posiiton of the arm 
   */
  public Pose getArmPose() {
    return armPose;
  }

  /**
   * Moves arm to a given pose 
   * @param pose the position that the arm is to go to 
   */
  public void moveToPose(Pose pose){
    armBase.moveBase((int)(0.5+ArmKinematics.getBaseRotation(pose)));
    armShoulder.moveShoulder((int)(0.5+ArmKinematics.getShoulderRotation(pose)));
    armExtender.set(ArmKinematics.getArmExtension(pose));
  }

  /**
   * Command factory that returns command that moves the arm to a given pose
   * @param pose the position that the arm is to go to
   * @return command that moves arm to a pose 
   */
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure arm update runs before you use arm pose values
    updateArmPose();
  }

  /**
   * Updates the arm pose based on sensor values
   */
  private void updateArmPose() {
    final double baseRotation = armBase.getAngle();
    final double sensedShoulderRotation = armShoulder.getAngle();
    final double shoulderRotation = correctShoulderAngle(sensedShoulderRotation);
    final double armExtension = armExtender.getLength();

    this.armPose = ArmKinematics.getPose(baseRotation, shoulderRotation, armExtension);
  }
}
