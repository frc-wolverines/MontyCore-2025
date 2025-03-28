package team5274.robot.subsystems.elevator;

import java.util.function.Supplier;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.Constants.ElevatorPivotConstants;
import team5274.robot.DeviceMap.ElevatorPivotMap;
import team5274.robot.subsystems.Superstructure.SuperPose;

public class ElevatorPivot extends SubsystemBase implements SubsystemFrame {
    public double cachedAngle = SuperPose.IDLE.elevatorAngle; //Cached angle to hold at
    private TalonFX master, slave;
    private DutyCycleEncoder encoder;

    private PIDController positionController;
    private PIDController persistantController;

    private static ElevatorPivot _instance;

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

        encoder = new DutyCycleEncoder(new DigitalInput(ElevatorPivotMap.kEncoderId.getDeviceId()));

        positionController = new PIDController(ElevatorPivotConstants.kP, ElevatorPivotConstants.kI, ElevatorPivotConstants.kD);
        positionController.enableContinuousInput(-Math.PI, Math.PI);
        positionController.setTolerance(ElevatorPivotConstants.kAngleTolerance);

        persistantController = new PIDController(ElevatorPivotConstants.kPP, ElevatorPivotConstants.kPI, ElevatorPivotConstants.kPD);
        persistantController.enableContinuousInput(-Math.PI, Math.PI);
        persistantController.setTolerance(ElevatorPivotConstants.kAngleTolerance);
        
        // setDefaultCommand(dutyCycleCommand(() -> -RobotContainer.operatorController.getLeftY()));
        // setDefaultCommand(run(() -> {}).withName("ElevatorPivot Idle"));
        setDefaultCommand(persistantAngleCommand(() -> cachedAngle));
    }

    /**
     * Retrieves the angle of the Elevator, relative to the Elevator being vertical
     * @return an angle in radians
     */
    public double getAngle() {
        return encoder.get() * 2 * Math.PI;
    }

    /**
     * Constructs a command to control the Elevator Pivot subsystem using duty-cycle control
     * @param dutyCycleSupplier a source of duty-cycle input
     * @return A command which controls the Elevator Pivot based on the dutyCycleSupplier's value
     */
    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return run(() -> master.setControl(new DutyCycleOut(dutyCycleSupplier.get()))).withName("Elevator Pivot Duty Cycle Command");
    }

    /**
     * Constructs a command to control the Elevator Pivot subsystem using a PID Controller to move to a given angle
     * @param targetAngle an angle in radians relative to the Elevator being vertical
     * @return A terminating command which moves the Elevator to a given angle
     */
    public Command angleCommand(double targetAngle) {
        return runEnd(
            () -> master.setControl(new DutyCycleOut(
                positionController.calculate(getAngle(), targetAngle)
            )),
            () -> master.stopMotor()
        ).until(
            positionController::atSetpoint
        ).beforeStarting(() -> cachedAngle = targetAngle, this).withName("ElevatorPivot Angle Command");
    }

    /**
     * Constructs a command to control the Elevator Pivot subsystem using a PID Controller to move to a given angle (Used as the default command)
     * @param targetAngle an angle in radians relative to the Elevator being vertical
     * @return A non-terminating command which moves the Elevator to a given angle and keeps it there
     */
    public Command persistantAngleCommand(double targetAngle) {
        return run(() -> master.setControl(new DutyCycleOut(
            persistantController.calculate(getAngle(), targetAngle)
        ))).withName("ElevatorPivot Persistant Angle Command");
    }

    /**
     * Constructs a command to control the Elevator Pivot subsystem using a PID Controller to move to a given angle (Used as the default command)
     * @param targetAngle an angle in radians relative to the Elevator being vertical
     * @return A non-terminating command which moves the Elevator to a given angle and keeps it there
     */
    public Command persistantAngleCommand(Supplier<Double> targetAngle) {
        return run(() -> master.setControl(new DutyCycleOut(
            persistantController.calculate(getAngle(), targetAngle.get())
        ))).withName("Elevator Pivot Persistant Angle Command");
    }

    @Override
    public void periodic() {
        if(!encoder.isConnected()) {
            master.disable();
            slave.disable();
        }
        sendTelemetry();
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);
        
        SmartDashboard.putNumber(this.getName() + "/Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(this.getName() + "/Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());

        SmartDashboard.putData(this.getName() + "/Through Bore Encoder", encoder);
        SmartDashboard.putNumber(this.getName() + "/Through Bore Encoder/Reported Angle", getAngle());

        SmartDashboard.putData(this.getName() + "/Position PID Controller", positionController);
        SmartDashboard.putNumber(getName() + "/Cached Angle", cachedAngle);

        SmartDashboard.putBoolean(this.getName() + "/Encoder Connected", encoder.isConnected());
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
