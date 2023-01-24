package frc.robot.positioning;

/**
 * The position class that was written before is exactly the same as the Pose
 * class written in another branch. Pose allows you to do horrible math easily.
 * It's probably best to use for things like trying to get the arm to a certain
 * place in the world.
 */
public class Position extends Pose {
    public Position(double x, double y, double rot){
        super(x, y, 0);
        rotate(0, 0, rot);
    }

    public double getRot() {
        return this.getZRot();
    }
}
