import frc.robot.RobotContainer;
import frc.robot.commands.autotasks.ExampleAutoTask;

import org.junit.jupiter.api.Test;

public class BaseTests {
  @Test
  public void TestTests() {
    System.out.println("Test :)");
    assert true;
  }

  @Test
  public void TestAutoTasks() {
    new ExampleAutoTask(null).verify();
  }
}
