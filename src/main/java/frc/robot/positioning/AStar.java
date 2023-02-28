// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.positioning;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicReference;

/*
 * A* (A Star) path planning uses a grid of Nodes, and assigns values to said Nodes to decide on the best route for the robot.
 * Quick Overview: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
 * In Depth: https://csis.pace.edu/~benjamin/teaching/cs627/webfiles/Astar.pdf
 * Stolen Code: https://github.com/Suwadith/A-Star-Shortest-Pathfinding-Algorithm-Square-Grid-Java/blob/FinalizedVersion/src/PathFindingOnSquaredGrid.java
 * Copy of this code, but executable https://github.com/Berdenson/A-Star-Shortest-Pathfinding-Algorithm-Square-Grid-Java
 */

/** A* class, used to generate a path for the trajectory. */
public class AStar {
  /** The map of nodes */
  private static Node[][] cell = new Node[Constants.AStar.FIELD_Y * 2][Constants.AStar.FIELD_X * 2];
  /** The pathlist, the path, the line to follow */
  private ArrayList<Node> pathList = new ArrayList<>();
  /** Nodes that no longer need to be aknowledged by the pathfinder */
  private ArrayList<Node> closedList = new ArrayList<>();
  /** The map of obstacles (true if obstacle present) */
  private static boolean[][] grid =
      new boolean[Constants.AStar.FIELD_Y * 2][Constants.AStar.FIELD_X * 2];
  /** Current path object the path generation algorithm is generating */
  private AtomicReference<Path> currPath = new AtomicReference<Path>(new Path());
  /** The path generation thread */
  private Thread genThread = new Thread();
  /** The y of the start coord */
  public int startY;
  /** The x of the start coord */
  public int startX;
  /** The y of the end coord */
  public int endY;
  /** The x of the end coord */
  public int endX;

  static {
    // creates nodes for cell
    for (int i = 0; i < Constants.AStar.FIELD_Y * 2; i++) {
      for (int j = 0; j < Constants.AStar.FIELD_X * 2; j++) {
        cell[i][j] = new Node(i, j);
      }
    }

    // creates the boolean obstacle matrix
    // TODO: Inputs the obstacles for the field (nonosquares), input your nono squares here.
    generateNoNoZone(10, 20, 10, 20); /* placeholder/example */
  }

  /** Creates a new pathfinding situation. Input should be in centimeters. Values are inclusive. */
  public AStar(int startX, int startY, int endX, int endY) {
    this.startX = startX;
    this.startY = startY;
    this.endX = endX;
    this.endY = endY;
  }

  /**
   * Run this off an object to generate a path using A*.
   *
   * @return An ArrayList of doubles[] containing an x and y value.
   */
  public AtomicReference<Path> generateAStarPath() {
    currPath.set(new Path());
    /* If the generation path is alive then dont generate another path */
    if (!genThread.isAlive()) {
      /* Reassign genthread with the code to generate a new thread */
      this.genThread =
          new Thread(
              () -> {

                // Runs generateHValue to get the pathlist.
                pathList =
                    generateHValue(
                        grid,
                        startY,
                        startX,
                        endY,
                        endX,
                        Constants.AStar.FIELD_Y * 2,
                        Constants.AStar.FIELD_X * 2,
                        10,
                        14);

                // Outputs whether or not the pathlist *is* a pathlist
                if (cell[startY][startX].hValue != -1 && pathList.contains(cell[endY][endX])) {

                  //TODO: THIS SHOULD OUTPUT TO SHUFFLEBOARD
                  System.out.println("Euclidean Path Found");

                } else {
                  // TODO: THIS SHOULD OUTPUT TO SHUFFLEBOARD
                  System.out.println("Euclidean Path Not found");
                }

                /*
                 * Converts the pathlist from Node(int cm) to double[] meters relative to the center (origin) of the field.
                 * Purely for the final return.
                 */
                ArrayList<Double[]> pathListInMeters = new ArrayList<Double[]>();
                for (Node node : pathList) {
                  Double tempNode[] = {
                    (double) (node.getY() - Constants.AStar.FIELD_X) / 100,
                    (double) (node.getX() - Constants.AStar.FIELD_Y) / 100
                  };
                  pathListInMeters.add(tempNode);
                }

                Collections.reverse(pathListInMeters);
                currPath.set(new Path(pathListInMeters));
              });
    }
    genThread.start();

    return currPath;
  }

  /**
   * Generates the Heuristic value. The Heuristic is the estimated distance from the current node
   * (not starting node) to the (start||end) node.
   *
   * @param matrix The boolean matrix that the framework generates
   * @param startY Starting point's x value
   * @param startX Starting point's y value
   * @param endY Ending point's x value
   * @param endX Ending point's y value
   * @param width Width of the field
   * @param length Length of the field
   * @param v Cost between 2 cells located horizontally or vertically next to each other
   * @param d Cost between 2 cells located Diagonally next to each other
   * @return The Pathlist of Nodes.
   */
  private ArrayList<Node> generateHValue(
      boolean matrix[][],
      int startY,
      int startX,
      int endY,
      int endX,
      int width,
      int length,
      int v,
      int d) {

    for (int i = 0; i < Constants.AStar.FIELD_Y * 2; i++) {
      for (int j = 0; j < Constants.AStar.FIELD_X * 2; j++) {
        // Checks whether a cell is Blocked or Not by checking the boolean value (true if obstacle
        // absent)
        if (!matrix[i][j]) {
          // Assigning the Euclidean Heuristic value
          cell[i][j].hValue = (int) (Math.pow(i - endY, 2) + Math.pow(j - endX, 2));
        } else {
          // If the boolean value is false (it's an obstacle), then assigning -1 instead of the
          // absolute length
          cell[i][j].hValue = -1;
        }
      }
    }
    return generatePath(
        cell,
        startY,
        startX,
        endY,
        endX,
        Constants.AStar.FIELD_Y * 2,
        Constants.AStar.FIELD_X * 2,
        v,
        d);
  }

  /**
   * Actually generates the pathlist.
   *
   * @param hValue Node type 2D Array (Matrix)
   * @param startY Starting point's y value
   * @param startX Starting point's x value
   * @param endY Ending point's y value
   * @param endX Ending point's x value
   * @param width Width of the field
   * @param length Length of the field
   * @param v Cost between 2 cells located horizontally or vertically next to each other
   * @param d Cost between 2 cells located Diagonally next to each other
   * @param additionalPath Boolean to decide whether to calculate the cost of through the diagonal
   *     path
   * @return The Pathlist of Nodes.
   */
  private ArrayList<Node> generatePath(
      Node hValue[][], int startY, int startX, int endY, int endX, int x, int y, int v, int d) {

    // Creation of a PriorityQueue and the declaration of the Comparator
    PriorityQueue<Node> openList =
        new PriorityQueue<Node>(
            11,
            new Comparator<Node>() {
              @Override
              // Compares 2 Node objects stored in the PriorityQueue and Reorders the Queue
              // according to the object which has the lowest fValue
              public int compare(Node cell1, Node cell2) {
                return cell1.fValue < cell2.fValue ? -1 : cell1.fValue > cell2.fValue ? 1 : 0;
              }
            });

    // Adds the Starting cell inside the openList
    openList.add(cell[startY][startX]);

    // Executes the rest if there are objects left inside the PriorityQueue
    while (true) {

      // Gets and removes the objects that's stored on the top of the openList and
      // saves it inside node
      Node node = openList.poll();

      // Checks if whether node is empty and f it is then breaks the while loop
      if (node == null) {
        break;
      }

      // Checks if whether the node returned is having the same node object values of
      // the ending point
      // If it des then stores that inside the closedList and breaks the while loop
      if (node == cell[endY][endX]) {
        closedList.add(node);
        break;
      }

      closedList.add(node);

      // Left Cell
      if (node.getX() != 0) {
        if (cell[node.getY()][node.getX() - 1].hValue != -1
            && !openList.contains(cell[node.getY()][node.getX() - 1])
            && !closedList.contains(cell[node.getY()][node.getX() - 1])) {
          int tCost = node.fValue + v;
          cell[node.getY()][node.getX() - 1].gValue = v;
          int cost = cell[node.getY()][node.getX() - 1].hValue + tCost;
          if (cell[node.getY()][node.getX() - 1].fValue > cost
              || !openList.contains(cell[node.getY()][node.getX() - 1]))
            cell[node.getY()][node.getX() - 1].fValue = cost;

          openList.add(cell[node.getY()][node.getX() - 1]);
          cell[node.getY()][node.getX() - 1].setParent(node);
        }
      }

      // Right Cell
      if (node.getX() != Constants.AStar.FIELD_X * 2 - 1) {
        if (cell[node.getY()][node.getX() + 1].hValue != -1
            && !openList.contains(cell[node.getY()][node.getX() + 1])
            && !closedList.contains(cell[node.getY()][node.getX() + 1])) {
          int tCost = node.fValue + v;
          cell[node.getY()][node.getX() + 1].gValue = v;
          int cost = cell[node.getY()][node.getX() + 1].hValue + tCost;
          if (cell[node.getY()][node.getX() + 1].fValue > cost
              || !openList.contains(cell[node.getY()][node.getX() + 1]))
            cell[node.getY()][node.getX() + 1].fValue = cost;

          openList.add(cell[node.getY()][node.getX() + 1]);
          cell[node.getY()][node.getX() + 1].setParent(node);
        }
      }

      // Bottom Cell
      if (node.getY() != Constants.AStar.FIELD_Y * 2 - 1) {
        if (cell[node.getY() + 1][node.getX()].hValue != -1
            && !openList.contains(cell[node.getY() + 1][node.getX()])
            && !closedList.contains(cell[node.getY() + 1][node.getX()])) {
          int tCost = node.fValue + v;
          cell[node.getY() + 1][node.getX()].gValue = v;
          int cost = cell[node.getY() + 1][node.getX()].hValue + tCost;
          if (cell[node.getY() + 1][node.getX()].fValue > cost
              || !openList.contains(cell[node.getY() + 1][node.getX()]))
            cell[node.getY() + 1][node.getX()].fValue = cost;

          openList.add(cell[node.getY() + 1][node.getX()]);
          cell[node.getY() + 1][node.getX()].setParent(node);
        }
      }

      // Top Cell
      if (node.getY() != 0) {
        if (cell[node.getY() - 1][node.getX()].hValue != -1
            && !openList.contains(cell[node.getY() - 1][node.getX()])
            && !closedList.contains(cell[node.getY() - 1][node.getX()])) {
          int tCost = node.fValue + v;
          cell[node.getY() - 1][node.getX()].gValue = v;
          int cost = cell[node.getY() - 1][node.getX()].hValue + tCost;
          if (cell[node.getY() - 1][node.getX()].fValue > cost
              || !openList.contains(cell[node.getY() - 1][node.getX()]))
            cell[node.getY() - 1][node.getX()].fValue = cost;

          openList.add(cell[node.getY() - 1][node.getX()]);
          cell[node.getY() - 1][node.getX()].setParent(node);
        }
      }

      // TopLeft Cell
      if (node.getY() != 0 && node.getX() != 0) {
        if (cell[node.getY() - 1][node.getX() - 1].hValue != -1
            && !openList.contains(cell[node.getY() - 1][node.getX() - 1])
            && !closedList.contains(cell[node.getY() - 1][node.getX() - 1])) {
          int tCost = node.fValue + d;
          cell[node.getY() - 1][node.getX() - 1].gValue = d;
          int cost = cell[node.getY() - 1][node.getX() - 1].hValue + tCost;
          if (cell[node.getY() - 1][node.getX() - 1].fValue > cost
              || !openList.contains(cell[node.getY() - 1][node.getX() - 1]))
            cell[node.getY() - 1][node.getX() - 1].fValue = cost;

          openList.add(cell[node.getY() - 1][node.getX() - 1]);
          cell[node.getY() - 1][node.getX() - 1].setParent(node);
        }
      }

      // TopRight Cell
      if (node.getY() != 0 && node.getX() != Constants.AStar.FIELD_X * 2 - 1) {
        if (cell[node.getY() - 1][node.getX() + 1].hValue != -1
            && !openList.contains(cell[node.getY() - 1][node.getX() + 1])
            && !closedList.contains(cell[node.getY() - 1][node.getX() + 1])) {
          int tCost = node.fValue + d;
          cell[node.getY() - 1][node.getX() + 1].gValue = d;
          int cost = cell[node.getY() - 1][node.getX() + 1].hValue + tCost;
          if (cell[node.getY() - 1][node.getX() + 1].fValue > cost
              || !openList.contains(cell[node.getY() - 1][node.getX() + 1]))
            cell[node.getY() - 1][node.getX() + 1].fValue = cost;

          openList.add(cell[node.getY() - 1][node.getX() + 1]);
          cell[node.getY() - 1][node.getX() + 1].setParent(node);
        }
      }

      // BottomLeft Cell
      if (node.getY() != Constants.AStar.FIELD_Y * 2 - 1 && node.getX() != 0) {
        if (cell[node.getY() + 1][node.getX() - 1].hValue != -1
            && !openList.contains(cell[node.getY() + 1][node.getX() - 1])
            && !closedList.contains(cell[node.getY() + 1][node.getX() - 1])) {
          int tCost = node.fValue + d;
          cell[node.getY() + 1][node.getX() - 1].gValue = d;
          int cost = cell[node.getY() + 1][node.getX() - 1].hValue + tCost;
          if (cell[node.getY() + 1][node.getX() - 1].fValue > cost
              || !openList.contains(cell[node.getY() + 1][node.getX() - 1]))
            cell[node.getY() + 1][node.getX() - 1].fValue = cost;

          openList.add(cell[node.getY() + 1][node.getX() - 1]);
          cell[node.getY() + 1][node.getX() - 1].setParent(node);
        }
      }

      // BottomRight Cell
      if (node.getY() != Constants.AStar.FIELD_Y * 2 - 1
          && node.getX() != Constants.AStar.FIELD_X * 2 - 1) {
        if (cell[node.getY() + 1][node.getX() + 1].hValue != -1
            && !openList.contains(cell[node.getY() + 1][node.getX() + 1])
            && !closedList.contains(cell[node.getY() + 1][node.getX() + 1])) {
          int tCost = node.fValue + d;
          cell[node.getY() + 1][node.getX() + 1].gValue = d;
          int cost = cell[node.getY() + 1][node.getX() + 1].hValue + tCost;
          if (cell[node.getY() + 1][node.getX() + 1].fValue > cost
              || !openList.contains(cell[node.getY() + 1][node.getX() + 1]))
            cell[node.getY() + 1][node.getX() + 1].fValue = cost;

          openList.add(cell[node.getY() + 1][node.getX() + 1]);
          cell[node.getY() + 1][node.getX() + 1].setParent(node);
        }
      }
    }

    /*
     * for (int i = 0; i < n; ++i) {
     * for (int j = 0; j < n; ++j) {
     * System.out.print(cell[i][j].fValue + "    ");
     * }
     * System.out.println();
     * }
     */

    // Assigns the last Object in the closedList to the endNode variable
    Node endNode = closedList.get(closedList.size() - 1);

    // Checks if whether the endNode variable currently has a parent Node. if it
    // doesn't then stops moving forward.
    // Stores each parent Node to the PathList so it is easier to trace back the
    // final path
    while (endNode.getParent() != null) {
      Node currentNode = endNode;
      pathList.add(currentNode);
      endNode = endNode.getParent();
    }

    pathList.add(cell[startY][startX]);
    // Clears the openList
    openList.clear();

    return pathList;
  }

  /**
   * A NoNoZone is a obstacle. Obstacle is in rectangle form. originX/originY is the topleft origin
   * of the rectangle; width/length is the size of the rectangle
   *
   * @param originY - the starting y point (origin) of the obstacle
   * @param originX - the starting x point (origin) of the obstacle
   * @param length - the length (y wise) of the rectangle relative to the origin
   * @param width - the width (x wise) of the rectangle relative to the origin
   */
  private static void generateNoNoZone(int originY, int originX, int length, int width) {
    for (int y = originY; y <= originY + length; y++) {
      for (int x = originX; x <= originX + width; x++) {
        grid[y][x] = true;
      }
    }
  }
}
