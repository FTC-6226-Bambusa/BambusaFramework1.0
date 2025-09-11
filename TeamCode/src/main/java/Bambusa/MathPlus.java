package Bambusa;

/*
 * This class provides useful functions that are not already in the Math class.
 * Special calculations (such as projection math) should be put here.
*/

public class MathPlus {
    // Finds T Of The Way From Value A To Value B
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    // Distance Functions
    public static double dist(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    public static double dist(double x1, double y1, double z1, double x2, double y2, double z2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2) + Math.pow(z2 - z1, 2));
    }
    public static double dist(double dx, double dy) {
        return Math.sqrt(dx * dx + dy * dy);
    }
    public static double dist(double dx, double dy, double dz) {
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
}
