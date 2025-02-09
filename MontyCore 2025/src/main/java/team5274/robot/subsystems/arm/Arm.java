package team5274.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.Constants.ArmConstants;
import team5274.robot.DeviceMap.ArmMap;
import team5274.robot.subsystems.Superstructure.SuperstructureGoal;

public class Arm extends SubsystemBase implements SubsystemFrame {
    private TalonFX pinionMotor;
    DutyCycleEncoder encoder;

    private PIDController positionController;

    private static Arm _instance;

    public static Arm get() {
        if(_instance == null) _instance = new Arm();
        return _instance;
    }

    public Arm() {
        pinionMotor = new TalonFX(0);
        encoder = new DutyCycleEncoder(new DigitalInput(ArmMap.kEncoderId.getDeviceId()));

        positionController = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

        setDefaultCommand(angleCommand(() -> SuperstructureGoal.IDLE.armAngle));
    }

    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return runEnd(
            () -> {
                pinionMotor.setControl(new DutyCycleOut(dutyCycleSupplier.get()));
            },
            () -> { 
                pinionMotor.setControl(new NeutralOut());
            }
        ).withName("Arm duty cycle");
    }

    public Command pidCommand(Supplier<Double> positionSupplier) {
        return runEnd(
            () -> {
                pinionMotor.setControl(new DutyCycleOut(positionController.calculate(positionSupplier.get())));
            }, 
            () -> {
                pinionMotor.setControl(new NeutralOut());
            }
        ).withName("Arm PID Control");
    }

    public Command angleCommand(Supplier<Double> angleSupplier) {
        return runEnd(
            () -> {
                pinionMotor.setControl(new DutyCycleOut(positionController.calculate(angleSupplier.get() * ArmConstants.kGearRatio)));
            }, 
            () -> {
                pinionMotor.setControl(new NeutralOut());
            }
        ).withName("Arm Angle Control");
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(this.getName() + "/Pinion Position Rotations", pinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Pinion Velocity Rotations", pinionMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putData(this.getName() + "/Through Bore Encoder", encoder);

        SmartDashboard.putData(this.getName() + "/Position PID Controller", positionController);
    }

    @Override
    public void zeroSensors() {
        pinionMotor.setPosition(0);
    }

    @Override
    public void stop() {
        pinionMotor.stopMotor();
    }
}