package frc.robot.commands.autotasks;

import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;
import frc.robot.positioning.AStar;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class MobilityBonus extends AutoTask {

  @Override
  public void initTask() {
    // TODO Auto-generated method stub

    // TODO Get boolean input from shuffleboard, for top or bottom exit
    if (false) {
      setTaskPosition(new UnknownPose());
    }

    else {
      setTaskPosition(new UnknownPose());
    }
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
