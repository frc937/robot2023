package frc.robot.positioning;

/**
 * https://www.khanacademy.org/math/algebra-home/alg-vectors
 */
public class Vector3D {
	private double x, y, z;

	/**
	 * Constructs a default vector at the origin
	 */
	public Vector3D() {
		this.x = 0;
		this.y = 0;
		this.z = 0;
	}

	public Vector3D(double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Vector addition
	 */
	public void addVector(Vector3D rhs) {
		this.addVector(rhs.getX(), rhs.getY(), rhs.getZ());
	}

  /**
   * Vector addition
   */
	public void addVector(double x, double y, double z) {
		this.x += x;
		this.y += y;
		this.z += z;
	}

	public double getX() {
		return this.x;
	}

	public double getY() {
		return this.y;
	}

	public double getZ() {
		return this.z;
	}
}
