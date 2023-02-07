// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * 
 * @param cell - the map of nodes
 * @param pathList - likely the path
 * @param closedList - nodes that no longer need to be aknowledged by the pathfinding
 * @param grid - the map of obstacles
 * @param startY - the y of the start coord
 * @param startX - the x of the start coord
 * @param endY - the y of the end coord
 * @param endX - the x of the end coord
 */
public class AStar {
    private static Node[][] cell;
    private ArrayList<Node> pathList = new ArrayList<>();
    private ArrayList<Node> closedList = new ArrayList<>();
    private static boolean[][] grid = new boolean[Constants.AStar.FIELD_X*2][Constants.AStar.FIELD_Y*2];
    public int startY;
    public int startX;
    public int endY;
    public int endX;
    static {
    // return a random N-by-N boolean matrix
    // TODO: THIS IS (I BELIEVE) WHERE WE INPUT THE OBSTACLES, SO, YA KNOW, **IMPORTANT** ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    for (int i = 0; i < Constants.AStar.FIELD_X*2; i++) {
        for (int j = 0; j < Constants.AStar.FIELD_Y*2; j++) {
            // HERE
            //Creating a new Node object for each and every Cell of the Grid (Matrix)
            cell[i][j] = new Node(i, j);
        }
    }
}
    
    public AStar(int startY, int startX, int endY, int endX) {

    }
    
    /**
     * @param matrix         The boolean matrix that the framework generates
     * @param startY         Starting point's x value
     * @param startX         Starting point's y value
     * @param endY           Ending point's x value
     * @param endX           Ending point's y value
     * @param width          Width of the field
     * @param length         Length of the field
     * @param v              Cost between 2 cells located horizontally or vertically next to each other
     * @param d              Cost between 2 cells located Diagonally next to each other
     */
    public ArrayList<Node> generateHValue(boolean matrix[][], int startY, int startX, int endY, int endX, int width, int length, int v, int d) {

        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix.length; j++) {
                
                //Checks whether a cell is Blocked or Not by checking the boolean value
                if (matrix[i][j]) {
                    //Assigning the Euclidean Heuristic value
                    // TODO: IF THE THING IS RUNNING SLOW, REMOVE THE Math.sqrt() TO MAYBE IMPROVE PROCESSING PROWER
                    cell[i][j].hValue = (int)Math.sqrt(Math.pow(i - endY, 2) + Math.pow(j - endX, 2));
                }
            }
        }
        return generatePath(cell, startY, startX, endY, endX, Constants.AStar.FIELD_X, Constants.AStar.FIELD_Y, v, d);
    }

    public ArrayList<Node> generateAStarPath() {
        int gCost = 0;
        /*int fCost = 0;*/

        //Creation of a Node type 2D array
        cell = new Node[Constants.AStar.FIELD_X][Constants.AStar.FIELD_Y];

        //Loop to find all 3 pathways and their relative Final Cost values

        generateHValue(grid, startY, startX, endY, endX, Constants.AStar.FIELD_X*2, Constants.AStar.FIELD_Y*2, 10, 14);

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

        return pathList; //TODO: MAKE THIS REUTRN A node[] of 
    }

    /**
     * @param hValue         Node type 2D Array (Matrix)
     * @param startY         Starting point's y value
     * @param startX         Starting point's x value
     * @param endY           Ending point's y value
     * @param endX           Ending point's x value
     * @param width          Width of the field
     * @param length         Length of the field
     * @param v              Cost between 2 cells located horizontally or vertically next to each other
     * @param d              Cost between 2 cells located Diagonally next to each other
     * @param additionalPath Boolean to decide whether to calculate the cost of through the diagonal path
     */
    public ArrayList<Node> generatePath(Node hValue[][], int startY, int startX, int endY, int endX, int x, int y, int v, int d) {
    
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
          if (cell[node.getX()][node.getY()-1].hValue != -1
                  && !openList.contains(cell[node.getX()][node.getY()-1])
                  && !closedList.contains(cell[node.getX()][node.getY()-1])) {
              int tCost = node.fValue + v;
              cell[node.getX()][node.getY()-1].gValue = v;
              int cost = cell[node.getX()][node.getY()-1].hValue + tCost;
              if (cell[node.getX()][node.getY()-1].fValue > cost || !openList.contains(cell[node.getX()][node.getY()-1]))
                  cell[node.getX()][node.getY()-1].fValue = cost;

              openList.add(cell[node.getX()][node.getY()-1]);
              cell[node.getX()][node.getY()-1].setParent(node);
          }
          } catch (IndexOutOfBoundsException e) {
      }
      
          //Right Cell
          try {
          if (cell[node.getX()][node.getY()+1].hValue != -1
                  && !openList.contains(cell[node.getX()][node.getY()+1])
                  && !closedList.contains(cell[node.getX()][node.getY()+1])) {
              int tCost = node.fValue + v;
              cell[node.getX()][node.getY()+1].gValue = v;
              int cost = cell[node.getX()][node.getY()+1].hValue + tCost;
              if (cell[node.getX()][node.getY()+1].fValue > cost || !openList.contains(cell[node.getX()][node.getY()+1]))
                  cell[node.getX()][node.getY()+1].fValue = cost;

              openList.add(cell[node.getX()][node.getY()+1]);
              cell[node.getX()][node.getY()+1].setParent(node);
          }
          } catch (IndexOutOfBoundsException e) {
      }
      
          //Bottom Cell
          try {
          if (cell[node.getX()+1][node.getY()].hValue != -1
                  && !openList.contains(cell[node.getX()+1][node.getY()])
                  && !closedList.contains(cell[node.getX()+1][node.getY()])) {
              int tCost = node.fValue + v;
              cell[node.getX()+1][node.getY()].gValue = v;
              int cost = cell[node.getX()+1][node.getY()].hValue + tCost;
              if (cell[node.getX()+1][node.getY()].fValue > cost || !openList.contains(cell[node.getX()+1][node.getY()]))
                  cell[node.getX()+1][node.getY()].fValue = cost;

              openList.add(cell[node.getX()+1][node.getY()]);
              cell[node.getX()+1][node.getY()].setParent(node);
          }
          } catch (IndexOutOfBoundsException e) {
      }
      
          //Top Cell
          try {
          if (cell[node.getX()-1][node.getY()].hValue != -1
                  && !openList.contains(cell[node.getX()-1][node.getY()])
                  && !closedList.contains(cell[node.getX()-1][node.getY()])) {
              int tCost = node.fValue + v;
              cell[node.getX()-1][node.getY()].gValue = v;
              int cost = cell[node.getX()-1][node.getY()].hValue + tCost;
              if (cell[node.getX()-1][node.getY()].fValue > cost || !openList.contains(cell[node.getX()-1][node.getY()]))
                  cell[node.getX()-1][node.getY()].fValue = cost;

              openList.add(cell[node.getX()-1][node.getY()]);
              cell[node.getX()-1][node.getY()].setParent(node);
          }
          } catch (IndexOutOfBoundsException e) {
      }
      
            //TopLeft Cell
            try {
            if (cell[node.getX()-1][node.getY()-1].hValue != -1
                    && !openList.contains(cell[node.getX()-1][node.getY()-1])
                    && !closedList.contains(cell[node.getX()-1][node.getY()-1])) {
                int tCost = node.fValue + d;
                cell[node.getX()-1][node.getY()-1].gValue = d;
                int cost = cell[node.getX()-1][node.getY()-1].hValue + tCost;
                if (cell[node.getX()-1][node.getY()-1].fValue > cost || !openList.contains(cell[node.getX()-1][node.getY()-1]))
                    cell[node.getX()-1][node.getY()-1].fValue = cost;

                openList.add(cell[node.getX()-1][node.getY()-1]);
                cell[node.getX()-1][node.getY()-1].setParent(node);
            }
            } catch (IndexOutOfBoundsException e) {
        }
        
            //TopRight Cell
            try {
            if (cell[node.getX()-1][node.getY()+1].hValue != -1
                    && !openList.contains(cell[node.getX()-1][node.getY()+1])
                    && !closedList.contains(cell[node.getX()-1][node.getY()+1])) {
                int tCost = node.fValue + d;
                cell[node.getX()-1][node.getY()+1].gValue = d;
                int cost = cell[node.getX()-1][node.getY()+1].hValue + tCost;
                if (cell[node.getX()-1][node.getY()+1].fValue > cost || !openList.contains(cell[node.getX()-1][node.getY()+1]))
                    cell[node.getX()-1][node.getY()+1].fValue = cost;

                openList.add(cell[node.getX()-1][node.getY()+1]);
                cell[node.getX()-1][node.getY()+1].setParent(node);
            }
            } catch (IndexOutOfBoundsException e) {
        }
        
            //BottomLeft Cell
            try {
            if (cell[node.getX()+1][node.getY()-1].hValue != -1
                    && !openList.contains(cell[node.getX()+1][node.getY()-1])
                    && !closedList.contains(cell[node.getX()+1][node.getY()-1])) {
                int tCost = node.fValue + d;
                cell[node.getX()+1][node.getY()-1].gValue = d;
                int cost = cell[node.getX()+1][node.getY()-1].hValue + tCost;
                if (cell[node.getX()+1][node.getY()-1].fValue > cost || !openList.contains(cell[node.getX()+1][node.getY()-1]))
                    cell[node.getX()+1][node.getY()-1].fValue = cost;

                openList.add(cell[node.getX()+1][node.getY()-1]);
                cell[node.getX()+1][node.getY()-1].setParent(node);
            }
            } catch (IndexOutOfBoundsException e) {
        }
        
            //BottomRight Cell
            try {
            if (cell[node.getX()+1][node.getY()+1].hValue != -1
                    && !openList.contains(cell[node.getX()+1][node.getY()+1])
                    && !closedList.contains(cell[node.getX()+1][node.getY()+1])) {
                int tCost = node.fValue + d;
                cell[node.getX()+1][node.getY()+1].gValue = d;
                int cost = cell[node.getX()+1][node.getY()+1].hValue + tCost;
                if (cell[node.getX()+1][node.getY()+1].fValue > cost || !openList.contains(cell[node.getX()+1][node.getY()+1]))
                    cell[node.getX()+1][node.getY()+1].fValue = cost;

                openList.add(cell[node.getX()+1][node.getY()+1]);
                cell[node.getX()+1][node.getY()+1].setParent(node);
            }
            } catch (IndexOutOfBoundsException e) {
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
      while (endNode.getParent() != null) {
          Node currentNode = endNode;
          pathList.add(currentNode);
          endNode = endNode.getParent();
      }
  
      pathList.add(cell[startY][startX]);
      //Clears the openList
      openList.clear();

      return pathList;
  
    }

}
