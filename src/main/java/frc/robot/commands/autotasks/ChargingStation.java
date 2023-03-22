package frc.robot.commands.autotasks;

import frc.robot.Constants;
import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;
import frc.robot.commands.Balance;

public class ChargingStation extends AutoTask {
  Balance balance;
  public ChargingStation(Balance balance) {
    this.balance = balance;
  }

  @Override
  public void initTask() {
    // TODO Auto-generated method stub
    // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
    getAStar().unGenerateNoNoZone(0, 0, 0, 0);
    setTaskPosition(new UnknownPose());
    setArrivedCommand(balance);
  }

  @Override
  public void arrived() {
    // TODO Auto-generated method stub
    // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
    getAStar().unGenerateNoNoZone(0, 0, 0, 0);
  }

  @Override
  public void fallback(Pose position) {
    // TODO Auto-generated method stub
    // TODO: THESE AREN'T ACTUAL VALUES (obviously) THESE NEED TO BE FANCY SCHMANCY VALUES FOR THE CHARGING STATION
    getAStar().unGenerateNoNoZone(0, 0, 0, 0);
  }

  @Override
  protected void update(Pose position) {
    // TODO Auto-generated method stub
    
  }
  


}
