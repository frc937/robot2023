package frc.robot.commands.autotasks;

import frc.robot.commands.MoveToPose;
import frc.robot.positioning.Pose;

public class PlaceCone extends AutoTask {
  private MoveToPose moveToPose;
  public PlaceCone(MoveToPose mToPose) {
    moveToPose = mToPose;
  }

  @Override
  public void initTask() {
    
    
  }

  @Override
  public void arrived() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void fallback(Pose position) {
    // TODO Auto-generated method stub
    
  }

  @Override
  protected void update(Pose position) {
    // TODO Auto-generated method stub
    
  }

  
}
