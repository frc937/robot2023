package frc.robot.positioning;

import java.util.ArrayList;

/** Path object for A star path planning. */
public class Path {
  private ArrayList<Double[]> pathList;
  private boolean generatedPath;

  /** @return the pathList */
  public ArrayList<Double[]> getPathList() {
    return this.pathList;
  }

  /** 
   * Checks if the path is generated
   * @return (true/false) (path is/isn't generated)
   */
  public boolean isPathGenerated() {
    return this.generatedPath;
  }

  public Path() {
    generatedPath = false;
  }

  public Path(ArrayList<Double[]> pathList) {
    this.pathList = pathList;
    this.generatedPath = true;
  }
}
