package team5274.lib.util;

public class ConditionalUitls {
    public static boolean withinTolerance(double value, double expected, double tolerance) {
        return value < expected + tolerance && value > expected - tolerance;
    }

    public static double binaryToDirection(boolean positive) {
        return positive ? 1.0 : 0.0;
    }

    public static double binaryToDirection(boolean positive, boolean negative) {
        return positive ? 1.0 : negative ? -1.0 : 0.0;
    }
}
