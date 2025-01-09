package frc.robot.subsystems.drive;

import javax.print.attribute.standard.PresentationDirection;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.core.io.IOContext;

//TODO: Writing Periodic outputs
//TODO: Telemetry/ Odometry
//TODO: Swerve Module State Interpretation/ Handling
//TODO: Value Unit Conversion
public class Module {

    private final TalonFX driveMotor, pivotMotor;
    private final CANcoder pivotEncoder;

    private IOContainer io = new IOContainer();
    
    public static class IOContainer {

        // Input
        public double pivotPosition = 0.0;

        // Output
        public double demandedPivotPosition = 0.0;
        public double demandedDrivePosition = 0.0;
        public double demandedDriveVelocity = 0.0;
        
    }
}
