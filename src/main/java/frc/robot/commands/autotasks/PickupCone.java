package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;

public class PickupCone extends AutoTask {

  @Override
  public void initTask() {
    // TODO Auto-generated method stub
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      setTaskPosition(new UnknownPose());
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      setTaskPosition(new UnknownPose());
    }

    // put fancy command here that does fancy pickup
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
