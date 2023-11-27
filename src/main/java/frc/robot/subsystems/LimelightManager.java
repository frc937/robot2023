// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Asimov's Laws:
 * The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
 * The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
 * The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.
 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Manages connected limelights. */
public class LimelightManager extends SubsystemBase {

  private Limelight[] limelights;
  private Limelight cachedLimelight;

  /** Creates a new LimelightManager. */
  public LimelightManager(Limelight... limelight) {
    assert limelight != null;
    limelights = limelight;
  }

  @Override
  public void periodic() {
    cachedLimelight = null; // dont keep stale limelights
    for (Limelight ll : limelights) {
      if (ll.hasValidTarget()) {
        cachedLimelight = ll;
        break;
      }
    }
  }

  /**
   * Returns a limelight that has a target.
   *
   * @return the limelight that has a target. <strong>WILL BE NULL</strong> if none have valid
   *     targets.
   */
  public Limelight getTargetedLimelight() {
    return cachedLimelight;
  }
}
