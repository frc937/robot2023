package frc.robot.commands.autotasks;

import frc.robot.positioning.Pose;
import frc.robot.positioning.UnknownPose;
import frc.robot.positioning.AStar;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class MobilityBonus extends AutoTask {
  /** Defines whether or not the bot will exit from above the charging station (top/true), or below (bottom/false) */
  boolean isTopExit;

  @Override
  public void initTask() {
    // TODO Auto-generated method stub

    isTopExit = SmartDashboard.getBoolean("Exit from Top?", true);

    // TODO Get boolean input from shuffleboard, for top or bottom exit
    if (isTopExit) {
      // make sure this one is the top exit V
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
