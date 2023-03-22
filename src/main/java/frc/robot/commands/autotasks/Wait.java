package frc.robot.commands.autotasks;

import frc.robot.Constants;
import frc.robot.positioning.Pose;
import java.util.concurrent.atomic.AtomicReference;

public class Wait extends AutoTask {
  Thread thread = new Thread();

  @Override
  public void initTask() {
    // TODO Auto-generated method stub
    // THIS LITERALLY JUST GIVES YOU THE ROBOTS CURRENT POSITION; IT'S BOILERPLATE
    setTaskPosition(Constants.RobotDimensions.CENTER_POSE.getWorldOriented(new Pose()));
    this.thread = new Thread(() -> {
      try {
        wait();
      } catch (InterruptedException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    });
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
