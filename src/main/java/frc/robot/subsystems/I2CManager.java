// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TCA9548A;

// import edu.wpi.first.wpilibj.Notifier;
// import com.revrobotics.ColorSensorV3;

public class I2CManager extends SubsystemBase {
  private final TCA9548A multiplexer;
  private final Rev2mDistanceSensor dist; /* This is a color sensor, but we only use it for distance */
  // private ColorSensorV3 color;
  private boolean currentRangeValid;
  private double currentRange;
  private Color currentColor;

  // private Notifier thread;

  /** Creates a new I2CManager. */
  public I2CManager() {
    multiplexer = new TCA9548A();
    multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);
    dist = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
    dist.setAutomaticMode(true);
    dist.setRangeProfile(RangeProfile.kLongRange);
    /*multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
    color = new ColorSensorV3(I2C.Port.kMXP);*/
    /* TODO: color/distance sensors MAY need additional config */

    /* We run stuff that needs to be done periodically in a thread instead of this.periodic()
     * because I2C calls take a hot minute and can cause loop overruns
     */
    /*thread = new Notifier(() -> {
      multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);
      currentRangeValid = dist.isRangeValid();
      currentRange = dist.getRange();
      multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
      currentColor = color.getColor();
    });

    thread.startPeriodic(0.02);*/
    /* Period @ 20ms, the default loop time for command-based */
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
    /* This subsystem does stuff periodically, but it does it in a thread because I2C calls can
     * take a while and will cause loop overruns (see the constructor of this class)
     */
    /*multiplexer.setBus(Constants.I2C.DISTANCE_SENSOR_MULTIPLEXER_PORT);*/
    currentRangeValid = dist.isRangeValid();
    currentRange = dist.getRange();
    /*multiplexer.setBus(Constants.I2C.COLOR_SENSOR_MULTIPLEXER_PORT);
    currentColor = color.getColor();*/
  }
}
