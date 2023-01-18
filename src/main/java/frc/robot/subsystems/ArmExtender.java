// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem represents the ArmExtender on the robot
 * 
 */
public class ArmExtender extends SubsystemBase {

  private WPI_TalonSRX winch;

  private Rev2mDistanceSensor lengthSensor;

  private double setpoint;

  /** Creates a new ArmExtender. 
   * Should be called once from {@link frc.robot.RobotContainer}
  */
  public ArmExtender() {
    winch = new WPI_TalonSRX(Constants.ArmConstants.ID_TALON_ARM_WINCH);
    lengthSensor = new Rev2mDistanceSensor(Port.kOnboard);
    lengthSensor.setAutomaticMode(true);
    setpoint = Constants.ArmConstants.MIN_LENGTH_ARM_EXTENDER;



  }

  /** If the LengthSensor has a valid reading this method will return the length of the arm from the shoulder to the claw
   * 
   * @return the length of the arm from the shoulder to the claw in inches
   */
  public double getLength() {
    if (lengthSensor.isRangeValid()) {
      return lengthSensor.getRange();

    } else {
      throw new IllegalStateException("range is not valid");

    }

  }

  /**
   * sets the setpoint for the arm extension
   * @param setpoint how far we want the arm to extend in inches from the shoulder to the claw
   */
  public void set(double setpoint) {
   this.setpoint = setpoint;
  }

  /**
   * directs arm towards setpoint
   * Since this is the periodic method, this is called every time the scheduler runs
   */
  @Override
  public void periodic() {
    if (Math.abs(setpoint - getLength()) >= Constants.ArmConstants.DONE_THRESHOLD_ARM_EXTENSION) {
      if (getLength() > setpoint) {
        winch.set(-1 * Constants.ArmConstants.SPEED_WINCH_ARM_EXTENSION);
        
      } else {
        winch.set(Constants.ArmConstants.SPEED_WINCH_ARM_EXTENSION);
      }
    }
  }
}
