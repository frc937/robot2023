package frc.robot.commands.autotasks;

import frc.robot.Constants;
import frc.robot.positioning.Pose;
import java.util.concurrent.atomic.AtomicReference;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wait extends AutoTask {
  /** The amount of time the bot should wait for in seconds */
  private double time;
  private Thread thread = new Thread();

  @Override
  public void initTask() {
    // TODO Auto-generated method stub
    // THIS LITERALLY JUST GIVES YOU THE ROBOTS CURRENT POSITION; IT'S BOILERPLATE
    setTaskPosition(Constants.RobotDimensions.CENTER_POSE.getWorldOriented(new Pose()));

    // Is divided by 1000 to convert from milliseconds to seconds.
    time = SmartDashboard.getNumber("Wait Time", 0.0) / 1000;

    this.thread = new Thread(() -> {
      try {
        wait((long) time);
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
