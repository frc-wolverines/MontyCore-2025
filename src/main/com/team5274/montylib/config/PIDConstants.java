package frc.montylib.config;

import edu.wpi.first.math.controller.PIDController;

/**A class that represents the constants associated with a PIDController */
public class PIDConstants {
    
    public double kP, kI, kD = 0.0;

    /**
     * Constructs a new instance of PIDConstants
     * @param kp the controller's proportional constant
     * @param ki the controller's integral constant
     * @param kd the controller's derivative constant
     */
    public PIDConstants(double kp, double ki, double kd) {
        this.kP = kp;
        this.kI = ki;
        this.kD = kd;
    }

    /**
     * Converts an instance of PIDConstants into a PIDController with the PIDConstants stored constants
     * @return a PIDController with constants from an instance of PIDConstants
     */
    public PIDController toController() {
        return new PIDController(kP, kI, kD);
    }
}
