package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.MoveToPose;
import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;
import frc.robot.subsystems.arm.ArmClaw;

public class PlaceGamePiece extends AutoTask {
  private MoveToPose moveToPose;
  /** Whether or not the object being picked up is a cone; false for cube */
  private boolean isCone;
  /** the location to place the cone at, starting from the top of the field */
  private int location;
  ArmClaw armClaw = new ArmClaw();
  MoveToPose moveToPose = new MoveToPose(null, null, null, null, null)
  public PlaceGamePiece(MoveToPose mToPose) {
    moveToPose = mToPose;
  }

  @Override
  public void initTask() {

    location = (int) SmartDashboard.getNumber("location", 1);
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      if (location == 1) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 2) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 3) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 4) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 5) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 6) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 7) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 8) {
        setTaskPosition(new UnknownPose());
      }
    }

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      if (location == 1) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 2) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 3) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 4) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 5) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 6) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 7) {
        setTaskPosition(new UnknownPose());
      }
      else if (location == 8) {
        setTaskPosition(new UnknownPose());
      }
    }

    isCone = SmartDashboard.getBoolean("isCone", true);

    if (isCone) {
      //put fancy cone command here
      setArrivedCommand(MoveToPose.andThen(ArmClaw.openClawCommand()));
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
