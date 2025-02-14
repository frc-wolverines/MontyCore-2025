package team5274.robot.subsystems.arm;

import java.util.function.Supplier;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.Constants.ArmConstants;
import team5274.robot.Constants.ElevatorPivotConstants;
import team5274.robot.DeviceMap.ArmMap;

public class Arm extends SubsystemBase implements SubsystemFrame {
    private double cachedArmAngle = 0.0;
    private double cachedWristAngle = 0.0;
    private TalonFX pinionMotor, wristMotor;
    DutyCycleEncoder encoder;

    private PIDController armPositionController;
    private PIDController wristPositionController;

    private static Arm _instance;

    public static Arm get() {
        if(_instance == null) _instance = new Arm();
        return _instance;
    }

    public Arm() {
        pinionMotor = new TalonFX(ArmMap.kPinionMotorId.getDeviceId());
        pinionMotor.getConfigurator().apply(ArmConstants.kPinionConfig);

        wristMotor = new TalonFX(ArmMap.kWristMotorId.getDeviceId());
        wristMotor.getConfigurator().apply(ArmConstants.kWristConfig);

        encoder = new DutyCycleEncoder(new DigitalInput(ArmMap.kEncoderId.getDeviceId()));

        armPositionController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);
        armPositionController.enableContinuousInput(-Math.PI, Math.PI);

        wristPositionController = new PIDController(ArmConstants.kWristP, ArmConstants.kWristI, ArmConstants.kWristD);
        wristPositionController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Retrieves the current angle of the Arm subsystem, relative to the arm being at its maximum rotation
     * @return an angle in radians
     */
    public double getArmAngle() {
        return encoder.get() * 2 * Math.PI;
    }

    /**
     * Retrieves the current angle of the Arm's wrist, relative to the end affector being horizontal
     * @return an angle in radians
     */
    public double getWristAngle() {
        return wristMotor.getRotorPosition().getValueAsDouble() * ArmConstants.kWristGearRatio * 2 * Math.PI;
    }

    /**
     * Constructs a command to control the Arm subsystem using duty-cycle control
     * @param armDutyCycleSupplier a source of duty-cycle input for the Arm (rack-and-pinion)
     * @param wristDutyCycleSupplier a source of duty-cycle input for the wrist
     * @return A command which controls the Arm based on two dutyCycleSuppliers' values
     */
    public Command dutyCycleCommand(Supplier<Double> armDutyCycleSupplier, Supplier<Double> wristDutyCycleSupplier) {
        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armDutyCycleSupplier.get()));
            wristMotor.setControl(new DutyCycleOut(wristDutyCycleSupplier.get()));
        }).withName("Arm Duty Cycle Command");
    }

    /**
     * Constructs a command to control the Arm subsystem using positional control
     * @param targetArmAngle an angle in radians relative to the maximum arm rotation
     * @return A terminating command which moves the Arm to a given rotation and keeps the wrist at it's current position
     */
    public Command angleCommand(double targetArmAngle) {
        cachedArmAngle = targetArmAngle;

        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), targetArmAngle)));
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), cachedWristAngle)));
        }).unless(
            () -> Math.abs(targetArmAngle - getArmAngle()) < ArmConstants.kArmAngleTolerance && Math.abs(cachedWristAngle - getWristAngle()) < ArmConstants.kWristAngleTolerance
        ).withName("Arm Angle Command");
    }

    /**
     * Constructs a command to control ONLY the Arm's wrist using positional control
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A terminating command which moves ONLY the Arm's wrist to a given position
     */
    public Command orientWrist(double targetWristAngle) {
        cachedWristAngle = targetWristAngle;

        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), cachedArmAngle)));
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), targetWristAngle)));
        }).unless(
            () -> Math.abs(cachedWristAngle - getWristAngle()) < ArmConstants.kWristAngleTolerance
        ).withName("Arm Orient Wrist Command");
    }

    /**
     * Constructs a command to control the Arm subsystem using positional control
     * @param targetArmAngle an angle in radians relative to the maximum arm rotation
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A terminating command which moves the Arm and the Arm's wrist to given rotations
     */
    public Command angleCommand(double targetArmAngle, double targetWristAngle) {
        cachedArmAngle = targetArmAngle;
        cachedWristAngle = targetWristAngle;

        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), targetArmAngle)));   `
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), targetWristAngle)));
        }).unless(
            () -> Math.abs(targetArmAngle - getArmAngle()) < ArmConstants.kArmAngleTolerance && Math.abs(targetWristAngle - getWristAngle()) < ArmConstants.kWristAngleTolerance
        ).withName("Arm Angle Command");
    }

    /**
     * Constructs a command to control the Arm subsystem using positional control (Used as the default command)
     * @param targetArmAngle an angle in radians relative to the maximum arm rotation
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A non-terminating command which moves the Arm and the Arm's wrist to given rotations and keeps them there
     */
    public Command persistantAngleCommand(double targetArmAngle, double targetWristAngle) {
        cachedArmAngle = targetArmAngle;
        cachedWristAngle = targetWristAngle;

        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), targetArmAngle)));
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), targetWristAngle)));
        }).withName("Arm Persistant Angle Command");
    }

    @Override
    public void periodic() {
        sendTelemetry();
    }
    
    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(getName() + "/Wrist Position Rotations", wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(getName() + "/Wrist Velocity Rotations", wristMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(this.getName() + "/Pinion Position Rotations", pinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Pinion Velocity Rotations", pinionMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putData(this.getName() + "/Through Bore Encoder", encoder);

        SmartDashboard.putData(this.getName() + "/Arm PID Controller", armPositionController);
        
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