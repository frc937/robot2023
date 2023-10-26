package frc.robot.positioning;

import java.util.ArrayList;

/** Return object for {@link positioning.Path.generateAStarPath} */
public class Path {
  private ArrayList<Double[]> pathList;
  private final boolean generatedPath;

  public ArrayList<Double[]> getPathList() {
    return this.pathList;
  }

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
