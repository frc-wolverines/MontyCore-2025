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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.ArmConstants;
import team5274.robot.Constants.ElevatorPivotConstants;
import team5274.robot.DeviceMap.ArmMap;
import team5274.robot.subsystems.Superstructure.SuperstructurePose;

public class Arm extends SubsystemBase implements SubsystemFrame {
    private double cachedArmAngle = SuperstructurePose.IDLE.armAngle;
    private double initArmAngle;
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
        armPositionController.setTolerance(ArmConstants.kArmAngleTolerance);

        wristPositionController = new PIDController(ArmConstants.kWristP, ArmConstants.kWristI, ArmConstants.kWristD);
        wristPositionController.setTolerance(ArmConstants.kWristAngleTolerance);

        zeroSensors();
        // setDefaultCommand(resistDriftFromInit());
        setDefaultCommand(persistantAngleCommand(() -> cachedArmAngle, () -> cachedWristAngle));
    }

    public void init() {
        setDefaultCommand(persistantAngleCommand(() -> cachedArmAngle, () -> cachedWristAngle));
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
        return (wristMotor.getPosition().getValueAsDouble() / 5) * 2 * Math.PI;
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
    public Command orientArm(double targetArmAngle) {
        return runEnd(
            () -> {
                pinionMotor.setControl(new DutyCycleOut(
                    Math.min(0.5, Math.max(armPositionController.calculate(getArmAngle(), targetArmAngle), -0.5))
                ));
                // wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), cachedWristAngle)));
            },
            () -> {
                pinionMotor.stopMotor();
            }
        ).until(
            () -> armPositionController.atSetpoint()
        ).beforeStarting(() -> cachedArmAngle = targetArmAngle, this).withName("Arm Only Angle Command");
    }

    /**
     * Constructs a command to control ONLY the Arm's wrist using positional control
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A terminating command which moves ONLY the Arm's wrist to a given position
     */
    public Command orientWrist(double targetWristAngle) {
        return runEnd(
            () -> {
                wristMotor.setControl(new DutyCycleOut(
                    Math.min(0.25, Math.max(wristPositionController.calculate(getWristAngle(), targetWristAngle), -0.25))
                ));
                // wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), cachedWristAngle)));
            },
            () -> {
                wristMotor.stopMotor();
            }
        ).until(
            () -> wristPositionController.atSetpoint()
        ).beforeStarting(() -> cachedWristAngle = targetWristAngle, this).withName("Wrist Only Angle Command");
    }

    /**
     * Constructs a command to control the Arm subsystem using positional control (Used as the default command)
     * @param targetArmAngle an angle in radians relative to the maximum arm rotation
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A non-terminating command which moves the Arm and the Arm's wrist to given rotations and keeps them there
     */
    public Command persistantAngleCommand(double targetArmAngle, double targetWristAngle) {
        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(
                Math.min(0.5, Math.max(armPositionController.calculate(getArmAngle(), targetArmAngle), -0.5))
            ));
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), targetWristAngle)));
        }).withName("Arm Persistant Angle Command");
    }

    /**
     * Constructs a command to control the Arm subsystem using positional control (Used as the default command)
     * @param targetArmAngle an angle in radians relative to the maximum arm rotation
     * @param targetWristAngle an angle in radians relative to the end affector being horizontal
     * @return A non-terminating command which moves the Arm and the Arm's wrist to given rotations and keeps them there
     */
    public Command persistantAngleCommand(Supplier<Double> targetArmAngle, Supplier<Double> targetWristAngle) {
        return run(() -> {
            pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), targetArmAngle.get())));
            wristMotor.setControl(new DutyCycleOut(wristPositionController.calculate(getWristAngle(), targetWristAngle.get())));
        }).withName("Arm Persistant Angle Command");
    }

    // public Command resistDriftFromInit() {
    //     return Commands.none();
    //     // return runEnd(
    //     //     () -> pinionMotor.setControl(new DutyCycleOut(armPositionController.calculate(getArmAngle(), initArmAngle))), 
    //     //     () -> pinionMotor.stopMotor()
    //     // ).beforeStarting(() -> initArmAngle = getArmAngle()).ignoringDisable(true).withName("Resist the Drift!");
    // }

    @Override
    public void periodic() {
        if(!encoder.isConnected()) {
            pinionMotor.disable();
        }
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
        SmartDashboard.putNumber(this.getName() + "/Arm Angle", getArmAngle());
        SmartDashboard.putNumber(this.getName() + "/Wrist Angle", getWristAngle());
        SmartDashboard.putNumber(this.getName() + "/Cached Arm Angle", cachedArmAngle);
        SmartDashboard.putNumber(this.getName() + "/Init Arm Angle", initArmAngle);

        SmartDashboard.putData(this.getName() + "/Arm PID Controller", armPositionController);
        SmartDashboard.putBoolean(this.getName() + "/Encoder Connected", encoder.isConnected());
    }

    @Override
    public void zeroSensors() {
        pinionMotor.setPosition(0);
        wristMotor.setPosition(0);
    }

    @Override
    public void stop() {
        pinionMotor.stopMotor();
        wristMotor.stopMotor();
    }
}