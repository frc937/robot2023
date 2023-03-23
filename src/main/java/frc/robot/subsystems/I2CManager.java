// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TCA9548A;

public class I2CManager extends SubsystemBase {
  private TCA9548A multiplexer;
  private Rev2mDistanceSensor dist;
  private ColorSensorV3 color;
  private boolean currentRangeValid;
  private double currentRange;
  private Color currentColor;

  /** Creates a new I2CManager. */
  public I2CManager() {
    multiplexer = new TCA9548A();
    dist = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP, Unit.kMillimeters, RangeProfile.kDefault);
    color = new ColorSensorV3(I2C.Port.kMXP);
    /* TODO: color/distance sensors MAY need additional config */
  }

  public boolean isRangeValid() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);
    /* This might not work because the thread that setEnabled(bool) starts might not get the range
     * of the distance sensor before we .getRange()
     */
    dist.setAutomaticMode(true);
    currentRangeValid = dist.isRangeValid();
    currentRange = dist.getRange();
    dist.setAutomaticMode(false);
    multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
    currentColor = color.getColor();
  }
}
