package frc.montylib.hardware;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

public class ExtTalonFX extends TalonFX {

    public static enum TalonControlMode {
        OPEN_LOOP,
        POSITION,
        VELOCITY
    }

    private TalonControlMode controlMode = TalonControlMode.OPEN_LOOP;

    public ExtTalonFX(int id) {
        super(id);
    }

    public void feedRaw(double input) {
        if(controlMode != TalonControlMode.OPEN_LOOP) controlMode = TalonControlMode.OPEN_LOOP;
        double power = Math.max(-1, Math.min(input, 1));
        set(power);
    }

    public void feedPosition(double position) {

    }

    public static class IOContainer {
        
        // Output
        public double demandedPosition = 0.0;
        public double demandedVelocity = 0.0;

    }
}
