package frc.robot.subsystems.drive;

import javax.print.attribute.standard.PresentationDirection;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.core.io.IOContext;

import edu.wpi.first.wpilibj.CAN;

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

    public Module(ModuleConfiguration configuration) {
        driveMotor = new TalonFX(configuration.driveMotorID);
        pivotMotor = new TalonFX(configuration.pivotMotorID);
        pivotEncoder = new CANcoder(configuration.canCoderID);
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }
    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }
    public static class ModuleConfiguration {
        public final int driveMotorID;
        public final int pivotMotorID;
        public final int canCoderID;

        public ModuleConfiguration(int driveMotorID, int pivotMotorID, int canCoderID) {
            this.driveMotorID = driveMotorID;
            this.pivotMotorID = pivotMotorID;
            this.canCoderID = canCoderID;
        }
    }
}
