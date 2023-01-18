// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmClaw extends SubsystemBase {


  private WPI_TalonSRX clawMotor;
  private Double setpoint;

  /* *********************************************************************************
   * TODO: REPLACE THIS WITH A REAL PRESSURE SENSOR OBJECT BEFORE WE USE IT ON THE BOT
   * *********************************************************************************
   */
  private double clawPressure;

  /** Creates a new ArmClaw. */

  public ArmClaw() {
    clawMotor = new WPI_TalonSRX(Constants.ArmConstants.ID_TALON_ARM_CLAW);
    setpoint = null;
  }
  public void openClaw() {
    clawMotor.set(Constants.ArmConstants.SPEED_ARM_CLAW);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (setpoint.equals(null)) {
      return;
    } else {
      if (Math.abs(setpoint - clawPressure) >= Constants.ArmConstants.DONE_THRESHOLD_ARM_CLAW) {
        if (clawPressure > setpoint) {
          clawMotor.set(-1 * Constants.ArmConstants.SPEED_ARM_CLAW);
        } else {
          clawMotor.set(Constants.ArmConstants.SPEED_ARM_CLAW);
        }
      }
    }
  }
}
