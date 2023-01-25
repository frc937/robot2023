// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Node extends SubsystemBase {
  /** Creates a new Node. */
  public Node(int x, int y) {
    this.x = x;
    this.y = y;
  }

  int x;
  int y;
  double hValue;
  int gValue;
  double fValue;
  Node parent;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
