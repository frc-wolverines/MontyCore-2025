package team5274.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.Constants.ElevatorPivotConstants;
import team5274.robot.DeviceMap.ElevatorPivotMap;

public class ElevatorPivot extends SubsystemBase implements SubsystemFrame {
    private TalonFX master, slave;

    public static ElevatorPivot _instance;

    public static ElevatorPivot get() {
        if(_instance == null) _instance = new ElevatorPivot();
        return _instance;
    }

    public ElevatorPivot() {
        master = new TalonFX(ElevatorPivotMap.kMasterMotorId.getDeviceId());
        master.getConfigurator().apply(ElevatorPivotConstants.kMasterConfig);

        slave = new TalonFX(ElevatorPivotMap.kSlaveMotorId.getDeviceId());
        slave.getConfigurator().apply(ElevatorPivotConstants.kSlaveConfig);
        slave.setControl(new Follower(master.getDeviceID(), true));
        
        setDefaultCommand(dutyCycleCommand(() -> 0.0));
    }

    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return startEnd(
            () -> {
                master.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
            },
            () -> { 
                master.setControl(new DutyCycleOut(0.0));
            }
        ).withName("Elevator Pivot duty cycle");
    }

    public Command motionMagicCommand(Supplier<Double> motionMagicPositionSupplier) {
        return startEnd(
            () -> {
                master.setControl(new MotionMagicDutyCycle(motionMagicPositionSupplier.get()));
            }, 
            () -> {
                master.setControl(new NeutralOut());
            }
        ).withName("Elevator Pivot MotionMagic");
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putNumber("Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Encoder Position Rotations", 0); //Configure
        SmartDashboard.putNumber("Encoder Position Angle", 0); //Configure
    }

    @Override
    public void zeroSensors() {
        master.setPosition(0);
        slave.setPosition(0);
    }

    @Override
    public void stop() {
        master.stopMotor();
        slave.stopMotor();
    }
}
