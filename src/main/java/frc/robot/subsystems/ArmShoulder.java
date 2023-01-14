// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmShoulder extends SubsystemBase {

  WPI_TalonSRX armShoulderMotor;

  /** Creates a new ArmShoulder. */
  public ArmShoulder() {
    armShoulderMotor = configTalon(Constants.ArmConstants.ID_TALON_ARM_SHOULDER);
  }
  private WPI_TalonSRX configTalon(int id) {
    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    //talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    talon.setSensorPhase(Constants.ArmConstants.INVERTED_TALON_SENSOR_ARM_SHOULDER);
    talon.setInverted(Constants.ArmConstants.INVERTED_TALON_ARM_SHOULDER);
    //talon.config_kP(0, Constants.ArmConstants.PID.kP);
    //talon.config_kI(0, Constants.ArmConstants.PID.kI);
    //talon.config_kD(0, Constants.ArmConstants.PID.kD);
    //talon.config_kF(0, Constants.ArmConstants.PID.kFF);

    return talon;

  }

  public void moveShoulder(int degrees) {
    degrees = (degrees / 360) * 4096;
    armShoulderMotor.set(ControlMode.Position, degrees);

  }

  public void stop() {
    armShoulderMotor.stopMotor();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

}
