// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.positioning;

/** One of the points on the path planning gridmap. */
public class Node {

  /* The logic behind private/public is that some values may be used in other classes, while hValue is unlikely to be used. */
  /** x coordinate on the field */
  private int x;
  /** y coordinate on the field */
  private int y;
  /**
   * The Heuristic value, distance of each node from (starting point || end point (idk which)) as
   * the crow flies
   */
  public int hValue;
  /** The g value, the distance between the current node and the start node */
  public int gValue;
  /** The f value, the total cost of the node */
  public int fValue;
  /** The parent node */
  private Node parent;

  /** Creates a new Node. Input should be in centimeters */
  public Node(int y, int x) {
    this.x = x;
    this.y = y;
  }

  /** Sets the Node's x value */
  public void setX(int x) {
    this.x = x;
  }

  /** Sets the Node's y value */
  public void setY(int y) {
    this.y = y;
  }

  /** Sets the Node's parent */
  public void setParent(Node parent) {
    this.parent = parent;
  }

  /**
   * @return The Node's x value
   */
  public int getX() {
    return this.x;
  }

  /**
   * @return The Node's y value
   */
  public int getY() {
    return this.y;
  }

  /**
   * @return The Node's parent
   */
  public Node getParent() {
    return this.parent;
  }
}
