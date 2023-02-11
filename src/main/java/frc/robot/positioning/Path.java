package frc.robot.positioning;

import java.util.ArrayList;

public class PathObject {
    private ArrayList<Double[]> pathList;
    private boolean generatedPath;

    public ArrayList<Double[]> getPathList() {
        return this.pathList;
    }

    public boolean isPathGenerated() {
        return this.generatedPath;
    }

    public PathObject() {
        generatedPath = false;
    }
    
    public PathObject(ArrayList<Double[]> pathList) {
        this.pathList = pathList;
        this.generatedPath = true;
    }
    

    
}
