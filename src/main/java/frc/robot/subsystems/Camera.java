// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  private int port;
  private UsbCamera camera;

  /** Creates a new Camera. */
  public Camera(int port) {
    this.port = port;
  }

  public void startCamera() {
    camera = CameraServer.startAutomaticCapture(this.port);

    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
