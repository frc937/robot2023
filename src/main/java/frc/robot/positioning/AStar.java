// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.positioning;

import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicReference;

/*
 * A* (A Star) path planning uses a grid of Nodes, and assigns values to said Nodes to decide on the best route for the robot.
 * Quick Overview: https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
 * In Depth: https://csis.pace.edu/~benjamin/teaching/cs627/webfiles/Astar.pdf
 * Stolen Code: https://github.com/Suwadith/A-Star-Shortest-Pathfinding-Algorithm-Square-Grid-Java/blob/FinalizedVersion/src/PathFindingOnSquaredGrid.java
 */

/** A* class, used to generate a path for the trajectory. */
public class AStar {
  /** The map of nodes */
  private static Node[][] cell;
  /** Likely the path */
  private ArrayList<Node> pathList = new ArrayList<>();
  /** Nodes that no longer need to be aknowledged by the pathfinder */
  private ArrayList<Node> closedList = new ArrayList<>();
  /** The map of obstacles (false if obstacle present) */
  private static boolean[][] grid =
      new boolean[Constants.AStar.FIELD_X * 2][Constants.AStar.FIELD_Y * 2];
  /** Current path object the path generation algorithm is generating */
  private AtomicReference<Path> currPath = new AtomicReference<Path>(new Path());
  /** The path generation thread */
  private Thread genThread;
  /** The y of the start coord */
  public int startY;
  /** The x of the start coord */
  public int startX;
  /** The y of the end coord */
  public int endY;
  /** The x of the end coord */
  public int endX;

  static {
    // creates the boolean obstacle matrix
    // TODO: Inputs the obstacles for the field (nonosquares), input your nono squares here.
    generateNoNoZone(69, 420, 69, 420); /* placeholder/example */
  }

  /** Creates a new pathfinding situation. Input should be in centimeters. */
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
                int gCost = 0;
                /* int fCost = 0; */

                // Creation of a Node type 2D array
                cell = new Node[Constants.AStar.FIELD_X][Constants.AStar.FIELD_Y];

                // Loop to find all 3 pathways and their relative Final Cost values
                pathList =
                    generateHValue(
                        grid,
                        startY,
                        startX,
                        endY,
                        endX,
                        Constants.AStar.FIELD_X * 2,
                        Constants.AStar.FIELD_Y * 2,
                        10,
                        14);

                if (cell[startY][startX].hValue != -1 && pathList.contains(cell[endY][endX])) {

                  /*
                   * This is only if you want to output the total cost of the whole thing; aka
                   * it's unnessecary.
                   */
                  // for (int i = 0; i < pathList.size() - 1; i++) {
                  // /* System.out.println(pathList.get(i).x + " " + pathList.get(i).y);*/
                  // gCost += pathList.get(i).gValue;
                  // /*fCost += pathList.get(i).fValue;*/
                  // }

                  System.out.println("Euclidean Path Found");
                  System.out.println("Total Cost: " + gCost / 10);
                  /* System.out.println("Total fCost: " + fCost); */
                  gCost = 0;
                  /* fCost = 0; */

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
                    (double) (node.getX() - Constants.AStar.FIELD_X) / 100,
                    (double) (node.getY() - Constants.AStar.FIELD_Y) / 100
                  };
                  pathListInMeters.add(tempNode);
                }

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

    for (int i = 0; i < matrix.length; i++) {
      for (int j = 0; j < matrix.length; j++) {

        // Checks whether a cell is Blocked or Not by checking the boolean value (true if obstacle
        // absent)
        if (matrix[i][j]) {
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
        cell, startY, startX, endY, endX, Constants.AStar.FIELD_X, Constants.AStar.FIELD_Y, v, d);
  }

  /**
   * Actually generates the path, in a 3x3 grid, with the center being the node.
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
      if (node.getY() != 0) {
        if (cell[node.getX()][node.getY() - 1].hValue != -1
            && !openList.contains(cell[node.getX()][node.getY() - 1])
            && !closedList.contains(cell[node.getX()][node.getY() - 1])) {
          int tCost = node.fValue + v;
          cell[node.getX()][node.getY() - 1].gValue = v;
          int cost = cell[node.getX()][node.getY() - 1].hValue + tCost;
          if (cell[node.getX()][node.getY() - 1].fValue > cost
              || !openList.contains(cell[node.getX()][node.getY() - 1]))
            cell[node.getX()][node.getY() - 1].fValue = cost;

          openList.add(cell[node.getX()][node.getY() - 1]);
          cell[node.getX()][node.getY() - 1].setParent(node);
        }
      }

      // Right Cell
      if (node.getY() != Constants.AStar.FIELD_X * 2 - 1) {
        if (cell[node.getX()][node.getY() + 1].hValue != -1
            && !openList.contains(cell[node.getX()][node.getY() + 1])
            && !closedList.contains(cell[node.getX()][node.getY() + 1])) {
          int tCost = node.fValue + v;
          cell[node.getX()][node.getY() + 1].gValue = v;
          int cost = cell[node.getX()][node.getY() + 1].hValue + tCost;
          if (cell[node.getX()][node.getY() + 1].fValue > cost
              || !openList.contains(cell[node.getX()][node.getY() + 1]))
            cell[node.getX()][node.getY() + 1].fValue = cost;

          openList.add(cell[node.getX()][node.getY() + 1]);
          cell[node.getX()][node.getY() + 1].setParent(node);
        }
      }

      // Bottom Cell
      if (node.getX() != Constants.AStar.FIELD_Y * 2 - 1) {
        if (cell[node.getX() + 1][node.getY()].hValue != -1
            && !openList.contains(cell[node.getX() + 1][node.getY()])
            && !closedList.contains(cell[node.getX() + 1][node.getY()])) {
          int tCost = node.fValue + v;
          cell[node.getX() + 1][node.getY()].gValue = v;
          int cost = cell[node.getX() + 1][node.getY()].hValue + tCost;
          if (cell[node.getX() + 1][node.getY()].fValue > cost
              || !openList.contains(cell[node.getX() + 1][node.getY()]))
            cell[node.getX() + 1][node.getY()].fValue = cost;

          openList.add(cell[node.getX() + 1][node.getY()]);
          cell[node.getX() + 1][node.getY()].setParent(node);
        }
      }

      // Top Cell
      if (node.getX() != 0) {
        if (cell[node.getX() - 1][node.getY()].hValue != -1
            && !openList.contains(cell[node.getX() - 1][node.getY()])
            && !closedList.contains(cell[node.getX() - 1][node.getY()])) {
          int tCost = node.fValue + v;
          cell[node.getX() - 1][node.getY()].gValue = v;
          int cost = cell[node.getX() - 1][node.getY()].hValue + tCost;
          if (cell[node.getX() - 1][node.getY()].fValue > cost
              || !openList.contains(cell[node.getX() - 1][node.getY()]))
            cell[node.getX() - 1][node.getY()].fValue = cost;

          openList.add(cell[node.getX() - 1][node.getY()]);
          cell[node.getX() - 1][node.getY()].setParent(node);
        }
      }

      // TopLeft Cell
      if (node.getX() != 0 && node.getY() != 0) {
        if (cell[node.getX() - 1][node.getY() - 1].hValue != -1
            && !openList.contains(cell[node.getX() - 1][node.getY() - 1])
            && !closedList.contains(cell[node.getX() - 1][node.getY() - 1])) {
          int tCost = node.fValue + d;
          cell[node.getX() - 1][node.getY() - 1].gValue = d;
          int cost = cell[node.getX() - 1][node.getY() - 1].hValue + tCost;
          if (cell[node.getX() - 1][node.getY() - 1].fValue > cost
              || !openList.contains(cell[node.getX() - 1][node.getY() - 1]))
            cell[node.getX() - 1][node.getY() - 1].fValue = cost;

          openList.add(cell[node.getX() - 1][node.getY() - 1]);
          cell[node.getX() - 1][node.getY() - 1].setParent(node);
        }
      }

      // TopRight Cell
      if (node.getX() != 0 && node.getY() != Constants.AStar.FIELD_X * 2 - 1) {
        if (cell[node.getX() - 1][node.getY() + 1].hValue != -1
            && !openList.contains(cell[node.getX() - 1][node.getY() + 1])
            && !closedList.contains(cell[node.getX() - 1][node.getY() + 1])) {
          int tCost = node.fValue + d;
          cell[node.getX() - 1][node.getY() + 1].gValue = d;
          int cost = cell[node.getX() - 1][node.getY() + 1].hValue + tCost;
          if (cell[node.getX() - 1][node.getY() + 1].fValue > cost
              || !openList.contains(cell[node.getX() - 1][node.getY() + 1]))
            cell[node.getX() - 1][node.getY() + 1].fValue = cost;

          openList.add(cell[node.getX() - 1][node.getY() + 1]);
          cell[node.getX() - 1][node.getY() + 1].setParent(node);
        }
      }

      // BottomLeft Cell //TODO: HI FUTURE BRADEN THIS IS ACCESSING FIELD Y FOR X CHECK THAT ITS
      // PROBABLY WRONG
      if (node.getX() != Constants.AStar.FIELD_Y * 2 - 1 && node.getY() != 0) {
        if (cell[node.getX() + 1][node.getY() - 1].hValue != -1
            && !openList.contains(cell[node.getX() + 1][node.getY() - 1])
            && !closedList.contains(cell[node.getX() + 1][node.getY() - 1])) {
          int tCost = node.fValue + d;
          cell[node.getX() + 1][node.getY() - 1].gValue = d;
          int cost = cell[node.getX() + 1][node.getY() - 1].hValue + tCost;
          if (cell[node.getX() + 1][node.getY() - 1].fValue > cost
              || !openList.contains(cell[node.getX() + 1][node.getY() - 1]))
            cell[node.getX() + 1][node.getY() - 1].fValue = cost;

          openList.add(cell[node.getX() + 1][node.getY() - 1]);
          cell[node.getX() + 1][node.getY() - 1].setParent(node);
        }
      }

      // BottomRight Cell
      if (node.getX() != Constants.AStar.FIELD_X * 2 - 1
          && node.getY() != Constants.AStar.FIELD_Y * 2 - 1) {
        if (cell[node.getX() + 1][node.getY() + 1].hValue != -1
            && !openList.contains(cell[node.getX() + 1][node.getY() + 1])
            && !closedList.contains(cell[node.getX() + 1][node.getY() + 1])) {
          int tCost = node.fValue + d;
          cell[node.getX() + 1][node.getY() + 1].gValue = d;
          int cost = cell[node.getX() + 1][node.getY() + 1].hValue + tCost;
          if (cell[node.getX() + 1][node.getY() + 1].fValue > cost
              || !openList.contains(cell[node.getX() + 1][node.getY() + 1]))
            cell[node.getX() + 1][node.getY() + 1].fValue = cost;

          openList.add(cell[node.getX() + 1][node.getY() + 1]);
          cell[node.getX() + 1][node.getY() + 1].setParent(node);
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
   * @param originX - the starting x point (origin) of the obstacle
   * @param originY - the starting y point (origin) of the obstacle
   * @param width - the width (x wise) of the rectangle relative to the origin
   * @param length - the length (y wise) of the rectangle relative to the origin
   */
  private static void generateNoNoZone(int originX, int originY, int width, int length) {
    for (int y = originY; y <= originY + length; y++) {
      for (int x = originX; x <= originX + length; x++) {
        grid[y][x] = true;
      }
    }
  }
}
