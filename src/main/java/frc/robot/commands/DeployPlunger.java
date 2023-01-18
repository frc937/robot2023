// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Plunger;
/** Command to deploy Plunger
 */
public class DeployPlunger extends CommandBase {
  private Plunger plunger;
  /**
   * Creates instance of DeployPlunger
   * @param plunger takes plunger subsystem for dependency injection  
   */
  public DeployPlunger(Plunger plunger) {
      this.plunger = plunger; 
      addRequirements(plunger);
  }

  /** Called when the command starts/button is presse - starts the motor */
  @Override
  public void initialize() {
    plunger.deployPlunger();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  /**called when the command end/button is realeased - stops the motor from spinning */
  @Override
  public void end(boolean interrupted) {
    plunger.stop();
  }

  /** Returns true when the command should end.
   * Will never return true due to because it's designed to be used with whenHeld
  */
  @Override
  public boolean isFinished() {
    return false;
  }
}
