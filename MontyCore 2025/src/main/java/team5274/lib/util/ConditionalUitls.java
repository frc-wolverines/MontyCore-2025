package team5274.lib.util;

public class ConditionalUitls {
    public static boolean withinTolerance(double value, double expected, double tolerance) {
        return value < expected + tolerance && value > expected - tolerance;
    }
}
