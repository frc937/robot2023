package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

/* This class was made to deploy the Plunger to keep
 * the bot in place on the Charge Station. 
 * It has two commands: a deploy command that tells the motor to turn 
 * and a stop command to tell the motor to stop turning. 
 */
public class Plunger {
        //defines theDeployer 
        Spark theDeployer = new Spark(1); 

    //code that deploys the Plunger
    public void deployPlunger() {
        theDeployer.set(0.25); 
    }

    //stops the motor from spinning
    public void stopPlunger() {
        theDeployer.set(0); 
    }
}
