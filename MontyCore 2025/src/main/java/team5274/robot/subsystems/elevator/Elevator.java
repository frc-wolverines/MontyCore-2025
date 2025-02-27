package team5274.robot.subsystems.elevator;

import java.util.function.Supplier;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.robot.Robot;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.DriveConstants;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.Constants.ElevatorPivotConstants;
import team5274.robot.DeviceMap.ElevatorMap;
import team5274.lib.control.SubsystemFrame;

public class Elevator extends SubsystemBase implements SubsystemFrame {
    private double cachedHeight = 0.0;
    private TalonFX master, slave;

    private PIDController controller;

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

        controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        controller.setTolerance(ElevatorConstants.kHeightTolerance);

        zeroSensors();
        setDefaultCommand(persistantHeightCommand(() -> cachedHeight));
        // setDefaultCommand(dutyCycleCommand(() -> -RobotContainer.operatorController.getRightY()));
    }

    /**
     * Retrieves the current height of the Elevator carriage, relative to the collapsed height of the carriage
     * @return a length in inches
     */
    public double getHeight() {
        return master.getRotorPosition().getValueAsDouble() * ElevatorConstants.kHeightRotationRatio;
    }

    /**
     * Constructs a command to control the Elevator subsystem using duty-cycle control
     * @param dutyCycleSupplier a source of duty-cycle input
     * @return A command which controls the Elevator based on the dutyCycleSupplier's value
     */
    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return run(() -> master.setControl(new DutyCycleOut(dutyCycleSupplier.get()))).withName("Elevator Duty Cycle Command");
    }

    /**
     * Constructs a command to control the Elevator subsystems using CTRE's built-in MotionMagic protocal to ascend to a given height
     * @param targetHeight a length in inches relative to the collapsed height of the carriage
     * @return A terminating command which moves the Elevator to a given height
     */
    public Command heightCommand(double targetHeight) {

        return runEnd(
            () -> master.setControl(new DutyCycleOut(
                controller.calculate(getHeight(), targetHeight)
            )),
            () -> master.stopMotor()
        ).until(
            controller::atSetpoint
        ).beforeStarting(() -> cachedHeight = targetHeight, this).withName("Elevator Height Command");

    }

    /**
     * Constructs a command to control the Elevator subsystems using CTRE's built-in MotionMagic protocal to ascend to a given height (Used as the default command)
     * @param targetHeight a length in inches relative to the collapsed height of the carriage
     * @return A non-terminating command which moves the Elevator to a given height and keep it there
     */
    public Command persistantHeightCommand(double targetHeight) {
        return run(() -> master.setControl(new DutyCycleOut(
            controller.calculate(getHeight(), targetHeight)
        ))).withName("Elevator Persistant Height Command");
    }

    /**
     * Constructs a command to control the Elevator subsystems using CTRE's built-in MotionMagic protocal to ascend to a given height (Used as the default command)
     * @param targetHeight a length in inches relative to the collapsed height of the carriage
     * @return A non-terminating command which moves the Elevator to a given height and keep it there
     */
    public Command persistantHeightCommand(Supplier<Double> targetHeight) {
        return run(() -> master.setControl(new DutyCycleOut(
            controller.calculate(getHeight(), targetHeight.get())
        ))).withName("Elevator Persistant Height Command");
    }

    @Override
    public void periodic() {
        sendTelemetry();
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(this.getName() + "/Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(this.getName() + "/Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(getName() + "Height", getHeight());
        SmartDashboard.putNumber(getName() + "Cached Height", cachedHeight);

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