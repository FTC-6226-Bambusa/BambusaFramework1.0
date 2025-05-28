package Bambusa.LimelightVision;

// Stores 3D point information. Example use in ObjectLocalizer.

public class Point3D {
    public double x; // Right (meters)
    public double y; // Forward (meters)
    public double z; // Up (meters)

    public Point3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString() {
        return String.format("X: %.3f m, Y: %.3f m, Z: %.3f m", x, y, z);
    }
}