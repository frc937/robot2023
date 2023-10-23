// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Manages connected limelights.
 */
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
    for (Limelight ll: limelights) {
      if (ll.hasValidTarget()) {
        cachedLimelight = ll;
        break;
      }
    }
  }

  /**
   * Returns a limelight that has a target.
   * @return the limelight that has a target. Null if none have valid targets.
   */
  public Limelight getTargetedLimelight() {
    return cachedLimelight;
  }
}
