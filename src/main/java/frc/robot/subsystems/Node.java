// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/**
 * @param x - x coordinate on the field 
 * @param y - y coordinate on the field
 * @param hValue - The Heuristic value, distance of each node from (starting point || end point (idk which))
 * @param gValue - The g value, the distance between the current node and the start node
 * @param fValue - The f value, the total cost of the node
 * @param parent - the parent Node
 */
public class Node {
  /** Creates a new Node. */
  public Node(int x, int y) {
    this.x = x;
    this.y = y;
  }

  int x;
  int y;
  int hValue;
  int gValue;
  int fValue;
  Node parent;
  static boolean[][] obstacle;
}
