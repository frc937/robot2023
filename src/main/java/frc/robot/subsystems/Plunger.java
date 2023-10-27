// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Plunger extends SubsystemBase {
  private Spark plunger;

  /** Creates a new Plunger. */
  public Plunger() {
    this.plunger = new Spark(Constants.Plunger.ID_PWMMOTOR_PLUNGER);
  }

  public void deployPlunger() {
    plunger.set(Constants.Plunger.SPEED_PLUNGER_DEPLOY);
  }

  public void stop() {
    plunger.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
