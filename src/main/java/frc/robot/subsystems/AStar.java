// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;

/**
 * @param pathList - 
 */
public class AStar extends SubsystemBase {
    static Node[][] cell;
    static ArrayList<Node> pathList = new ArrayList<>();
    static ArrayList<Node> closedList = new ArrayList<>();
    static boolean additionalPath = false;

    // return a random N-by-N boolean matrix
    // TODO: THIS IS (I BELIEVE) WHERE WE INPUT THE OBSTACLES, SO, YA KNOW, **IMPORTANT** ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public static boolean[][] createObstacleGrid(int N) {
        boolean[][] a = new boolean[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                // a[i][j] = StdRandom.bernoulli(p);
            }
        }
        return a;
    }

    /**
     * @param matrix         The boolean matrix that the framework generates
     * @param startY             Starting point's x value
     * @param startX             Starting point's y value
     * @param endY             Ending point's x value
     * @param endX             Ending point's y value
     * @param n              Length of one side of the matrix
     * @param v              Cost between 2 cells located horizontally or vertically next to each other
     * @param d              Cost between 2 cells located Diagonally next to each other
     * @param additionalPath Boolean to decide whether to calculate the cost of through the diagonal path
     */
    public static void generateHValue(boolean matrix[][], int startY, int startX, int endY, int endX, int n, int v, int d, boolean additionalPath) {

        for (int y = 0; y < matrix.length; y++) {
            for (int x = 0; x < matrix.length; x++) {
                //Creating a new Node object for each and every Cell of the Grid (Matrix)
                cell[y][x] = new Node(y, x);
                //Checks whether a cell is Blocked or Not by checking the boolean value
                if (matrix[y][x]) {
                    //Assigning the Euclidean Heuristic value
                    cell[y][x].hValue = Math.sqrt(Math.pow(y - endY, 2) + Math.pow(x - endX, 2));
                }
            }
        }
        generatePath(cell, startY, startX, endY, endX, n, v, d, additionalPath);
    }

    public static void menu() {
        int n = 69; //TODO: THIS IS A PLACEHOLDER VALUE, SET GRID SIZE LATER OR SOMETHING IDK ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        int gCost = 0;
        /*int fCost = 0;*/

        //Generating a new Boolean Matrix according to the input values of n and p (Length, Percolation value)
        boolean[][] grid = createObstacleGrid(n);

        //Creation of a Node type 2D array
        cell = new Node[grid.length][grid.length];

        // TODO: THIS IS WHERE YOU PUT THE VALUES FOR WHERE THE GOOD 'OL START AND END POS COORDS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        int startY = 69;
        int startX = 69;
        int endY = 69;
        int endX = 69;

        //Loop to find all 3 pathways and their relative Final Cost values

        generateHValue(grid, startY, startX, endY, endX, n, 10, 14, true);

        if (cell[startY][startX].hValue!=-1&&pathList.contains(cell[endY][endX])) {

            for (int i = 0; i < pathList.size() - 1; i++) {
                /* System.out.println(pathList.get(i).x + " " + pathList.get(i).y);*/
                /*StdDraw.circle(pathList.get(i).y, n - pathList.get(i).x - 1, .4);*/
                gCost += pathList.get(i).gValue;
                /*fCost += pathList.get(i).fValue;*/
            }
        
            System.out.println("Euclidean Path Found");
            System.out.println("Total Cost: " + gCost/10.0);
            /*System.out.println("Total fCost: " + fCost);*/
            gCost = 0;
            /*fCost = 0;*/
        
        } else {
            System.out.println("Euclidean Path Not found");
        }

        pathList.clear();
        closedList.clear();
    }

/**
 * @param hValue         Node type 2D Array (Matrix)
 * @param startY         Starting point's y value
 * @param startX         Starting point's x value
 * @param endY           Ending point's y value
 * @param endX           Ending point's x value
 * @param n              Length of one side of the matrix
 * @param v              Cost between 2 cells located horizontally or vertically next to each other
 * @param d              Cost between 2 cells located Diagonally next to each other
 * @param additionalPath Boolean to decide whether to calculate the cost of through the diagonal path
 */
public static void generatePath(Node hValue[][], int startY, int startX, int endY, int endX, int n, int v, int d, boolean additionalPath) {

  //Creation of a PriorityQueue and the declaration of the Comparator
  PriorityQueue<Node> openList = new PriorityQueue<>(11, new Comparator() {
      @Override
      //Compares 2 Node objects stored in the PriorityQueue and Reorders the Queue according to the object which has the lowest fValue
      public int compare(Object cell1, Object cell2) {
          return ((Node) cell1).fValue < ((Node) cell2).fValue ? -1 :
                  ((Node) cell1).fValue > ((Node) cell2).fValue ? 1 : 0;
      }
  });

  //Adds the Starting cell inside the openList
  openList.add(cell[startY][startX]);

  //Executes the rest if there are objects left inside the PriorityQueue
  while (true) {

      //Gets and removes the objects that's stored on the top of the openList and saves it inside node
      Node node = openList.poll();

      //Checks if whether node is empty and f it is then breaks the while loop
      if (node == null) {
          break;
      }

      //Checks if whether the node returned is having the same node object values of the ending point
      //If it des then stores that inside the closedList and breaks the while loop
      if (node == cell[endY][endX]) {
          closedList.add(node);
          break;
      }

      closedList.add(node);

      //Left Cell
      try {
          if (cell[node.x][node.y - 1].hValue != -1
                  && !openList.contains(cell[node.x][node.y - 1])
                  && !closedList.contains(cell[node.x][node.y - 1])) {
              double tCost = node.fValue + v;
              cell[node.x][node.y - 1].gValue = v;
              double cost = cell[node.x][node.y - 1].hValue + tCost;
              if (cell[node.x][node.y - 1].fValue > cost || !openList.contains(cell[node.x][node.y - 1]))
                  cell[node.x][node.y - 1].fValue = cost;

              openList.add(cell[node.x][node.y - 1]);
              cell[node.x][node.y - 1].parent = node;
          }
      } catch (IndexOutOfBoundsException e) {
      }

      //Right Cell
      try {
          if (cell[node.x][node.y + 1].hValue != -1
                  && !openList.contains(cell[node.x][node.y + 1])
                  && !closedList.contains(cell[node.x][node.y + 1])) {
              double tCost = node.fValue + v;
              cell[node.x][node.y + 1].gValue = v;
              double cost = cell[node.x][node.y + 1].hValue + tCost;
              if (cell[node.x][node.y + 1].fValue > cost || !openList.contains(cell[node.x][node.y + 1]))
                  cell[node.x][node.y + 1].fValue = cost;

              openList.add(cell[node.x][node.y + 1]);
              cell[node.x][node.y + 1].parent = node;
          }
      } catch (IndexOutOfBoundsException e) {
      }

      //Bottom Cell
      try {
          if (cell[node.x + 1][node.y].hValue != -1
                  && !openList.contains(cell[node.x + 1][node.y])
                  && !closedList.contains(cell[node.x + 1][node.y])) {
              double tCost = node.fValue + v;
              cell[node.x + 1][node.y].gValue = v;
              double cost = cell[node.x + 1][node.y].hValue + tCost;
              if (cell[node.x + 1][node.y].fValue > cost || !openList.contains(cell[node.x + 1][node.y]))
                  cell[node.x + 1][node.y].fValue = cost;

              openList.add(cell[node.x + 1][node.y]);
              cell[node.x + 1][node.y].parent = node;
          }
      } catch (IndexOutOfBoundsException e) {
      }

      //Top Cell
      try {
          if (cell[node.x - 1][node.y].hValue != -1
                  && !openList.contains(cell[node.x - 1][node.y])
                  && !closedList.contains(cell[node.x - 1][node.y])) {
              double tCost = node.fValue + v;
              cell[node.x - 1][node.y].gValue = v;
              double cost = cell[node.x - 1][node.y].hValue + tCost;
              if (cell[node.x - 1][node.y].fValue > cost || !openList.contains(cell[node.x - 1][node.y]))
                  cell[node.x - 1][node.y].fValue = cost;

              openList.add(cell[node.x - 1][node.y]);
              cell[node.x - 1][node.y].parent = node;
          }
      } catch (IndexOutOfBoundsException e) {
      }

      if (additionalPath) {

          //TopLeft Cell
          try {
              if (cell[node.x - 1][node.y - 1].hValue != -1
                      && !openList.contains(cell[node.x - 1][node.y - 1])
                      && !closedList.contains(cell[node.x - 1][node.y - 1])) {
                  double tCost = node.fValue + d;
                  cell[node.x - 1][node.y - 1].gValue = d;
                  double cost = cell[node.x - 1][node.y - 1].hValue + tCost;
                  if (cell[node.x - 1][node.y - 1].fValue > cost || !openList.contains(cell[node.x - 1][node.y - 1]))
                      cell[node.x - 1][node.y - 1].fValue = cost;

                  openList.add(cell[node.x - 1][node.y - 1]);
                  cell[node.x - 1][node.y - 1].parent = node;
              }
          } catch (IndexOutOfBoundsException e) {
          }

          //TopRight Cell
          try {
              if (cell[node.x - 1][node.y + 1].hValue != -1
                      && !openList.contains(cell[node.x - 1][node.y + 1])
                      && !closedList.contains(cell[node.x - 1][node.y + 1])) {
                  double tCost = node.fValue + d;
                  cell[node.x - 1][node.y + 1].gValue = d;
                  double cost = cell[node.x - 1][node.y + 1].hValue + tCost;
                  if (cell[node.x - 1][node.y + 1].fValue > cost || !openList.contains(cell[node.x - 1][node.y + 1]))
                      cell[node.x - 1][node.y + 1].fValue = cost;

                  openList.add(cell[node.x - 1][node.y + 1]);
                  cell[node.x - 1][node.y + 1].parent = node;
              }
          } catch (IndexOutOfBoundsException e) {
          }

          //BottomLeft Cell
          try {
              if (cell[node.x + 1][node.y - 1].hValue != -1
                      && !openList.contains(cell[node.x + 1][node.y - 1])
                      && !closedList.contains(cell[node.x + 1][node.y - 1])) {
                  double tCost = node.fValue + d;
                  cell[node.x + 1][node.y - 1].gValue = d;
                  double cost = cell[node.x + 1][node.y - 1].hValue + tCost;
                  if (cell[node.x + 1][node.y - 1].fValue > cost || !openList.contains(cell[node.x + 1][node.y - 1]))
                      cell[node.x + 1][node.y - 1].fValue = cost;

                  openList.add(cell[node.x + 1][node.y - 1]);
                  cell[node.x + 1][node.y - 1].parent = node;
              }
          } catch (IndexOutOfBoundsException e) {
          }

          //BottomRight Cell
          try {
              if (cell[node.x + 1][node.y + 1].hValue != -1
                      && !openList.contains(cell[node.x + 1][node.y + 1])
                      && !closedList.contains(cell[node.x + 1][node.y + 1])) {
                  double tCost = node.fValue + d;
                  cell[node.x + 1][node.y + 1].gValue = d;
                  double cost = cell[node.x + 1][node.y + 1].hValue + tCost;
                  if (cell[node.x + 1][node.y + 1].fValue > cost || !openList.contains(cell[node.x + 1][node.y + 1]))
                      cell[node.x + 1][node.y + 1].fValue = cost;

                  openList.add(cell[node.x + 1][node.y + 1]);
                  cell[node.x + 1][node.y + 1].parent = node;
              }
          } catch (IndexOutOfBoundsException e) {
          }
      }
  }

  /*for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
          System.out.print(cell[i][j].fValue + "    ");
      }
      System.out.println();
  }*/

  //Assigns the last Object in the closedList to the endNode variable
  Node endNode = closedList.get(closedList.size() - 1);

  //Checks if whether the endNode variable currently has a parent Node. if it doesn't then stops moving forward.
  //Stores each parent Node to the PathList so it is easier to trace back the final path
  while (endNode.parent != null) {
      Node currentNode = endNode;
      pathList.add(currentNode);
      endNode = endNode.parent;
  }

  pathList.add(cell[startY][startX]);
  //Clears the openList
  openList.clear();

  System.out.println();

}

public AStar() {

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
