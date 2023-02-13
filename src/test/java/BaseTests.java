import org.junit.jupiter.api.Test;

import frc.robot.RobotContainer;

public class BaseTests {
    @Test
    public void TestTests(){
        System.out.println("Test :)");
        assert true;
    }
    @Test
    public void TestAutoTasks(){
        new RobotContainer();
    }
}
