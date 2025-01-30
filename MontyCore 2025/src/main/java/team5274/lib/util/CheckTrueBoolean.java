package team5274.lib.util;

/** A boolean tht only returns true if it's iterative value changes from false to true */
public class CheckTrueBoolean {
    private boolean lastVal = false;

    public boolean update(boolean newVal) {
        boolean result = newVal && !lastVal;
        lastVal = newVal;
        return result;
    }
}
