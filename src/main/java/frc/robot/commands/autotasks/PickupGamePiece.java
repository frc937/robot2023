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
  /** The height of the game piece spot thing, 1 is the top, 3 is the bottom */
  private int location;
  /** Whether or not the object being picked up is a cone; false for cube */
  private boolean isCone;

  @Override
  public void initTask() {
    
    location = (int) SmartDashboard.getNumber("middleGamePieces", 3.0);

    if (location == 1) {
      spot = Spot.TOP;
    }
    else if (location == 2) {
      spot = Spot.MIDDLE;
    }
    else if (location == 3) {
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
