// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.SensorCollection;

/**
 * Subsystem that represents the "shoulder" of the arm; that is, the motor which will rotate the arm
 * extender. Can rotate to a given setpoint.
 */
public class ArmShoulder extends SubsystemBase {

  private WPI_TalonSRX armShoulderMotor;
  private SensorCollection sensorCollection;

  /** Creates a new ArmShoulder. Should be called once from {@link frc.robot.RobotContainer}. */
  public ArmShoulder() {
    armShoulderMotor = configTalon(Constants.Arm.ID_TALON_ARM_SHOULDER);
    sensorCollection = armShoulderMotor.getSensorCollection();
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
    talon.configClearPositionOnLimitR(Constants.Arm.AUTO_ZERO_REVERSE_LIMIT_SHOULDER, 0);
    talon.setSensorPhase(Constants.Arm.INVERTED_TALON_SENSOR_ARM_SHOULDER);
    talon.setInverted(Constants.Arm.INVERTED_TALON_ARM_SHOULDER);
    talon.config_kP(0, Constants.Arm.ShoulderPID.kP);
    talon.config_kI(0, Constants.Arm.ShoulderPID.kI);
    talon.config_kD(0, Constants.Arm.ShoulderPID.kD);
    talon.config_kF(0, Constants.Arm.ShoulderPID.kFF);
    talon.config_IntegralZone(0, Constants.Arm.ShoulderPID.kIZone);
    talon.configAllowableClosedloopError(0, Constants.Arm.ShoulderPID.ACCEPTABLE_ERROR);

    return talon;
  }

  public void manualMoveArmShoulder(double x) {
    armShoulderMotor.set(ControlMode.PercentOutput, x);
  }

  /** Returns the angle of the shoulder in degrees */
  public double getAngle() {
    return (armShoulderMotor.getSelectedSensorPosition() / 4096) * 360;
  }

  /**
   * Moves the shoulder to a given setpoint.
   *
   * @param degrees The setpoint to move to in degrees.
   */
  public void moveShoulder(int degrees) {
    /* Takes the degree param and converts it to encoder ticks
     * so the talon knows what we're talking about
     */
    degrees = (((degrees / 360) * 4096));
    armShoulderMotor.set(ControlMode.Position, degrees);
  }

  public boolean isShoulderAtSetpoint() {
    return armShoulderMotor.getSelectedSensorPosition() <= Constants.Arm.ShoulderPID.ACCEPTABLE_ERROR;
  }

  /** Checks if the reverse shoulder limit switch is closed
   * @return True if the switch is closed
   */
  public boolean shoulderLimitSwitch() {
    return sensorCollection.isRevLimitSwitchClosed();
  }

  /** Stops the shoulder from moving. */
  public void stop() {
    armShoulderMotor.stopMotor();
  }

  /** Subsystem periodic; called every scheduler run. Not used in this subsystem. */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
