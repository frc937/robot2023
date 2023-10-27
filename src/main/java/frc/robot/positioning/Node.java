// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.positioning;

/**
 * One of the points on the path planning gridmap.
 *
 * @param x - x coordinate on the field
 * @param y - y coordinate on the field
 * @param hValue - The Heuristic value, distance of each node from (starting point || end point (idk
 *     which))
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

  /* The logic behind private/public is that some values may be used in other classes, while hValue is unlikely to be used. */
  private int x;
  private int y;
  public int hValue;
  public int gValue;
  public int fValue;
  private Node parent;

  public void setX(int x) {
    this.x = x;
  }

  public void setY(int y) {
    this.y = y;
  }

  public void setParent(Node parent) {
    this.parent = parent;
  }

  public int getX() {
    return this.x;
  }

  public int getY() {
    return this.y;
  }

  public Node getParent() {
    return this.parent;
  }
}
