package frc.robot.commands.autotasks;

public class Position {
    private double x;
    private double y;
    private double rot;

    public Position(double x, double y, double rot){
        this.x = x;
        this.y = y;
        this.rot = rot;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }

    public double getRot(){
        return rot;
    }


}
