package team5274.robot.subsystems.elevator;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.DeviceMap.ElevatorMap;
import team5274.robot.subsystems.Superstructure.SuperstructureGoal;
import team5274.lib.control.SubsystemFrame;

public class Elevator extends SubsystemBase implements SubsystemFrame {
    private TalonFX master, slave;

    private static Elevator _instance;

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

        setDefaultCommand(heightCommand(() -> SuperstructureGoal.IDLE.elevatorHeight));
    }

    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return runEnd(
            () -> {
                master.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
            },
            () -> { 
                master.setControl(new DutyCycleOut(0.0));
            }
        ).withName("Elevator duty cycle");
    }

    public Command motionMagicCommand(Supplier<Double> motionMagicPositionSupplier) {
        return runEnd(
            () -> {
                master.setControl(new MotionMagicDutyCycle(motionMagicPositionSupplier.get()));
            }, 
            () -> {
                master.setControl(new NeutralOut());
            }
        ).withName("Elevator MotionMagic");
    }

    public Command heightCommand(Supplier<Double> heightSupplier) {
        return runEnd(
            () -> {
                master.setControl(new MotionMagicDutyCycle(heightSupplier.get() * ElevatorConstants.kGearRatio));
            }, 
            () -> {
                master.setControl(new NeutralOut());
            }
        ).withName("Elevator Height MotionMagic");
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(this.getName() + "/Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(this.getName() + "/Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());
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