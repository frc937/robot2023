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

public class ArmExtender extends SubsystemBase {

  private WPI_TalonSRX winch;

  private Rev2mDistanceSensor lengthSensor;


  /** Creates a new ArmExtender. */
  public ArmExtender() {
    winch = new WPI_TalonSRX(Constants.ArmConstants.ID_TALON_ARM_WINCH);
    lengthSensor = new Rev2mDistanceSensor(Port.kOnboard);
    lengthSensor.setAutomaticMode(true);

  }

  /*private double getLength() {
    if (lengthSensor.isRangeValid()) {
      return lengthSensor.getRange();

    }

  }*/

  public void moveArm(float setpoint) {
    //winch.set(Constants.)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
