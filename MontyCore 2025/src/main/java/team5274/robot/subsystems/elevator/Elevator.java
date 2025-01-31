package team5274.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.DeviceMap.ElevatorMap;
import team5274.lib.control.SubsystemFrame;

/** Add your docs here. */
public class Elevator extends SubsystemBase implements SubsystemFrame {
    private double heightObjective = ElevatorConstants.kMinHeight;
    private TalonFX master, slave;

    public static Elevator _instance;

    public static Elevator get() {
        if(_instance == null) _instance = new Elevator();
        return _instance;
    }

    public Elevator() {
        master = new TalonFX(ElevatorMap.kMasterMotorId.getDeviceId());
        master.getConfigurator().apply(ElevatorConstants.kMasterConfig);

        slave = new TalonFX(ElevatorMap.kSlaveMotorId.getDeviceId());
        slave.getConfigurator().apply(ElevatorConstants.kSlaveConfig);
        slave.setControl(new Follower(master.getDeviceID(), true));
    }

    public void dutyCycleCommand(double input) {
        startEnd(
            () -> {
                master.setControl(new DutyCycleOut(input));
            },
            () -> {
                master.setControl(new DutyCycleOut(0.0));
            }
        ).withName("Elevator duty cycle");;
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putNumber("Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());
    }

    @Override
    public void zeroSensors() {
        master.setPosition(0.0);
        slave.setPosition(0.0);
    }

    @Override
    public void stop() {
        master.stopMotor();
        slave.stopMotor();
    }
}