package frc.robot.commands.autotasks;

import frc.robot.commands.Balance;
import frc.robot.positioning.Pose;

public class GoToBalance extends AutoTask {
  private Balance balanceCommand;
  
  public GoToBalance(Balance balance) {
      balanceCommand = balance;
  }

  @Override
  public void initTask() {
    setUnknownLocation();
    setArrivedCommand(balanceCommand);
    
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
