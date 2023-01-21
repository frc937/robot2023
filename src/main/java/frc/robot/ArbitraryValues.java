// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Add your docs here. */
public class ArbitraryValues {

  /**
   * arbitrary value for unknown doubles; intended for values that we don't know due to unbuilt bot; DONT USE FOR ACTUAL BOT
   * @return 0.0 if the robot base isn't real; will throw exception if base is real
   */
  public static double arbitraryDouble(){
    if(RobotBase.isReal()){
      throw new IllegalStateException("unsafe value for use on bot");
    }
    return 0.0; 
  }

  /**
   * arbitrary value for unknown integer; intended for value that we don't know due to unbuilt bot; DON'T USE FOR ACTUAL BOT
   * @return 0 if the robot base isn't real; will throw exception if base is real
   */
  public static int arbitraryInteger(){
    if(RobotBase.isReal()){
      throw new IllegalStateException("unsafe value for use on bot");
    }
    return 0; 
  }
  
  /**
   * arbitrary value for unknown boolean; intended for value that we don't know due to unbuilt bot; DON'T USE FOR ACTUAL BOT
   * @return false if the robot base isn't real; will throw exception if base is real 
   */
  public static boolean arbitraryBoolean(){
    if(RobotBase.isReal()){
      throw new IllegalStateException("unsafe value for use on bot");
    }
    return false; 
  }
}
