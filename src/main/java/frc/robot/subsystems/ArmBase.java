// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that represents the base of the arm, which (if mechanical implements it)
 * will be able to rotate to a given setpoint.
 */
public class ArmBase extends SubsystemBase {

  private WPI_TalonSRX armBaseMotor;

  /** Creates a new ArmBase. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmBase() {
    armBaseMotor = configTalon(Constants.ArmConstants.ID_TALON_ARM_BASE);
  }

  /* TODO: Consider moving this method to a static class. It's not super useful here or in the other arm classes. */
  /**
   * Configures the Talon SRX for this class with values supplied in {@link frc.robot.Constants.ArmConstants}.
   * @param id The CAN ID of the Talon SRX
   * @return The newly constructed Talon SRX, configured and ready for PID
   */
  private WPI_TalonSRX configTalon(int id) {
    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    //talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    talon.setSensorPhase(Constants.ArmConstants.INVERTED_TALON_SENSOR_ARM_BASE);
    talon.setInverted(Constants.ArmConstants.INVERTED_TALON_ARM_BASE);
    //talon.config_kP(0, Constants.ArmConstants.BasePID.kP);
    //talon.config_kI(0, Constants.ArmConstants.BasePID.kI);
    //talon.config_kD(0, Constants.ArmConstants.BasePID.kD);
    //talon.config_kF(0, Constants.ArmConstants.BasePID.kFF);

    return talon;
  }

  /**
   * Moves the base to a given setpoint.
   * @param degrees The setpoint to move to in degrees.
   */
  public void moveBase(int degrees) {
    degrees = (degrees / 360) * 4096;
    armBaseMotor.set(ControlMode.Position, degrees);
  }

  /**
   * Stops the base from moving.
   */
  public void stop() {
    armBaseMotor.stopMotor();
  }

  /**
   * Subsystem periodic; called every scheduler run.
   * Not used in this subsystem.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
