package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MoveToPose;
import frc.robot.positioning.Pose;

public class PlaceGamePiece extends AutoTask {
  private MoveToPose moveToPose;
  public PlaceGamePiece(MoveToPose mToPose) {
    moveToPose = mToPose;
  }

  @Override
  public void initTask() {
    //TODO

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
