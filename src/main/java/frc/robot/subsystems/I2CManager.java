// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TCA9548A;

public class I2CManager extends SubsystemBase {
  private TCA9548A multiplexer;
  private ColorSensorV3 dist; /* This is a color sensor, but we only use it for distance */
  private ColorSensorV3 color;
  private boolean currentRangeValid;
  private double currentRange;
  private Color currentColor;

  /** Creates a new I2CManager. */
  public I2CManager() {
    multiplexer = new TCA9548A();
    multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);
    dist = new ColorSensorV3(I2C.Port.kMXP);
    multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
    color = new ColorSensorV3(I2C.Port.kMXP);
    /* TODO: color/distance sensors MAY need additional config */
  }

  public boolean isCurrentRangeValid() {
    return this.currentRangeValid;
  }

  public double getCurrentRange() {
    return this.currentRange;
  }

  public Color getCurrentColor() {
    return this.currentColor;
  }

  @Override
  public void periodic() {
    multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);
    currentRangeValid = dist.getProximity() == 0;
    currentRange = dist.getProximity(); /* TODO: convert between this and inches */
    /* TODO: remove smartdash stuff when we're done testing */
    SmartDashboard.putNumber("Current proximity", currentRange);
    multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
    currentColor = color.getColor();
    SmartDashboard.putNumber("Red:", color.getRed());
    SmartDashboard.putNumber("Blue", color.getBlue());
    SmartDashboard.putNumber("Green:", color.getGreen());
  }
}
