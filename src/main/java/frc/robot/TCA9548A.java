// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class TCA9548A {
  int portNum;
  I2C multiplexer;

  public TCA9548A(int portNum) {
    if (portNum > 7 || portNum < 0) return;
    this.portNum = 0x70 + portNum;
    multiplexer = new I2C(Port.kMXP, 0x70 + portNum);
  }

  public TCA9548A() {
    this(0); /* Default, this is the port this year's bot uses */
  }

  public void selectBus(int busNumber) {
    if (busNumber >= 8 || busNumber < 0) return;
    multiplexer.write(portNum, 1 << busNumber);
  }
}
