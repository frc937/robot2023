package frc.robot.commands.autotasks;

import frc.robot.positioning.Pose;

public class ExampleAutoTask extends AutoTask{
    public ExampleAutoTask(){
    }
    public void initTask(){
    }
    public boolean initFinished(){
        return true;
    }
    public void arrived(){
        
    }
    public boolean arrivedFinished(){
        return true;
    }
    public void fallback(Pose position){}
    public void update(Pose position){}
}
