package frc.robot.commands.autotasks;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;
import frc.robot.commands.Balance;

public class ChargingStation extends AutoTask {
  private Balance balance;
  public ChargingStation(Balance balance) {
    this.balance = balance;
  }

  @Override
  public void initTask() {

    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {

      // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
      getAStar().unGenerateNoNoZone(0, 0, 0, 0);
      setTaskPosition(new UnknownPose());
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {

      // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
      getAStar().unGenerateNoNoZone(0, 0, 0, 0);
      setTaskPosition(new UnknownPose());
    }
    setArrivedCommand(balance);
  }

  @Override
  public void arrived() {

    // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      getAStar().generateNoNoZone(0, 0, 0, 0);
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      getAStar().generateNoNoZone(0, 0, 0, 0);
    }
    
  }

  @Override
  public void fallback(Pose position) {

    // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
    if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      getAStar().generateNoNoZone(0, 0, 0, 0);
    }
    else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      getAStar().generateNoNoZone(0, 0, 0, 0);
    }
  }

  @Override
  protected void update(Pose position) {
    // TODO Auto-generated method stub
    
  }
  


}
