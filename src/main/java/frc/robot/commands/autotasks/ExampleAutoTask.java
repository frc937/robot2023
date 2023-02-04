package frc.robot.commands.autotasks;

import frc.robot.commands.ExampleCommand;
import frc.robot.positioning.Pose;
// TODO: write documentation
// TODO: finish example autotask
public class ExampleAutoTask extends AutoTask {

    private ExampleCommand exampleCommand;
    /**
         * Use the constructor to configure the autotask arrival and init commands
         * The order you run {@link #addArrivedCommand(edu.wpi.first.wpilibj2.command.CommandBase) addArrivedCommand} and
         * {@link #addInitCommand(edu.wpi.first.wpilibj2.command.CommandBase) addArrivedCommand} 
         * is the order the commands are ran. <p>
         * Dont create commands in autotasks, Use dependency injection. <p>
         * For a command to be valid {@link #setTaskPosition(Pose)} has to be ran. <p>
         * In the future this will be tested and the code will not build if there is an error
     * @param exampleCommand The commands the tasks needs, this one only needs the example command
     */
    public ExampleAutoTask(ExampleCommand exampleCommand){
        this.exampleCommand = exampleCommand;
        addArrivedCommand(exampleCommand); 
        setTaskPosition(new Pose()); // Normaly the pose would not be empty but this is a example so its okay.
    }
    /**
     * The methods below this are mainly for updates, like if you wanted to put somthing on smartdash you would put that in initTask.
     */
    public void initTask(){
    }
    public boolean initFinished(){
        return true;
    }
    public void arrived(){
        
    }
    public boolean arrivedFinished(){
        return true;
    }
    public void fallback(Pose position){}
    protected void update(Pose position){}
}
