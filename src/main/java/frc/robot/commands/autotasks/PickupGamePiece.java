package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;

public class PickupGamePiece extends AutoTask {

  private enum Spot {
    TOP,
    MIDDLE,
    BOTTOM
  };
  private Spot spot;
  private boolean isTop;
  private boolean isMiddle;
  private boolean isBottom;
  /** Whether or not the object being picked up is a cone; false for cube */
  private boolean isCone;

  @Override
  public void initTask() {
    
    isTop = SmartDashboard.getBoolean("exitTop", false);
    isMiddle = SmartDashboard.getBoolean("exitMiddle", false);
    isBottom = SmartDashboard.getBoolean("exitBottom", false);

    if (isTop) {
      spot = Spot.TOP;
    }
    else if (isMiddle) {
      spot = Spot.MIDDLE;
    }
    else if (isBottom) {
      spot = Spot.BOTTOM;
    }

    if (spot == Spot.TOP) {

      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        setTaskPosition(new UnknownPose());
      }
      else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        setTaskPosition(new UnknownPose());
      }
    }
    else if (spot == Spot.MIDDLE) {

      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        setTaskPosition(new UnknownPose());
      }
      else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        setTaskPosition(new UnknownPose());
      }
    }
    else if (spot == Spot.BOTTOM) {
      
      if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        setTaskPosition(new UnknownPose());
      }
      else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        setTaskPosition(new UnknownPose());
      }
    }

    isCone = SmartDashboard.getBoolean("isCone", true);

    if (isCone) {
      //put fancy cone command here
    }
    else {
      //put fancy cube command here
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
