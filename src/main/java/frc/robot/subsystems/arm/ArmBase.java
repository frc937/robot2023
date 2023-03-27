// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;


/**
 * Subsystem that represents the base of the arm, which (if mechanical implements it) will be able
 * to rotate to a given setpoint.
 */
public class ArmBase extends SubsystemBase {

  private WPI_TalonSRX armBaseMotor;
  private double uniBaseDegrees;
  private double baseRotation;
  private boolean isTripped;
  private SensorCollection sensorCollection;
  

  /** Creates a new ArmBase. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmBase() {
    armBaseMotor = configTalon(Constants.Arm.ID_TALON_ARM_BASE);
    sensorCollection = armBaseMotor.getSensorCollection();
  }

  /* TODO: Consider moving this method to a static class. It's not super useful here or in the other arm classes. */
  /**
   * Configures the Talon SRX for this class with values supplied in {@link
   * frc.robot.Constants.Arm}.
   *
   * @param id The CAN ID of the Talon SRX
   * @return The newly constructed Talon SRX, configured and ready for PID
   */
  private WPI_TalonSRX configTalon(int id) {
    /* Comment the stuff in this method that's commented out back in
     * when we have PID tuned and PID values are set in constants.
     */
    WPI_TalonSRX talon = new WPI_TalonSRX(id);
    // talon.configFactoryDefault();
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    talon.configClearPositionOnLimitR(Constants.Arm.AUTO_ZERO_REVERSE_LIMIT_BASE, 0);
    talon.setSensorPhase(Constants.Arm.INVERTED_TALON_SENSOR_ARM_BASE);
    talon.setInverted(Constants.Arm.INVERTED_TALON_ARM_BASE);
    talon.config_kP(0, Constants.Arm.BasePID.kP);
    talon.config_kI(0, Constants.Arm.BasePID.kI);
    talon.config_kD(0, Constants.Arm.BasePID.kD);
    talon.config_kF(0, Constants.Arm.BasePID.kFF);
    talon.config_IntegralZone(0, Constants.Arm.BasePID.kIZone);
    talon.configAllowableClosedloopError(0, Constants.Arm.BasePID.ACCEPTABLE_ERROR);


    

    return talon;
  }
  /** Checks if either base limit switch is closed. */
  public boolean baseLimitSwitch() {
    return sensorCollection.isFwdLimitSwitchClosed() || sensorCollection.isRevLimitSwitchClosed();
  }


  

  

  public void manualMoveArmBase(double x) {
    armBaseMotor.set(ControlMode.PercentOutput, x);
  }
  /** Returns the angle of the base CCW from forward in degrees */
  public double getAngle() {
    /* TODO: account for difference between limit switch position and front of bot */
    return (armBaseMotor.getSelectedSensorPosition() / 4096) * 360;
  }

  /**
   * Moves the base to a given setpoint.
   *
   * @param degrees The setpoint to move to in degrees.
   */
  public void moveBase(int degrees) {
    /* Takes the degree param and converts it to encoder ticks
     * so the talon knows what we're talking about
     */
    /* TODO: account for difference between limit switch position and front of bot
     * doing so will require some funky coding
     */
    degrees = (degrees / 360) * 4096;
    armBaseMotor.set(ControlMode.Position, degrees);
    uniBaseDegrees = degrees;
  }
  
  /** Checks if the base motor is at the setpoint.
   */
  public boolean isBaseAtSetpoint() {
    return armBaseMotor.getClosedLoopError() <= Constants.Arm.BasePID.ACCEPTABLE_ERROR;
  }

  /** Stops the base from moving. */
  public void stop() {
    armBaseMotor.stopMotor();
  }

  /** Subsystem periodic; called every scheduler run. Not used in this subsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
