package team5274.robot.subsystems.arm;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SignalsConfig;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.lib.util.ConditionalUitls;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.AffectorConstants;
import team5274.robot.DeviceMap.PincerMap;
import team5274.robot.subsystems.Superstructure.SuperPoseType;

public class Affector extends SubsystemBase implements SubsystemFrame {
    private SparkMax intakeMotor;

    private static Affector _instance;

    public static Affector get() {
        if(_instance == null) _instance = new Affector();
        return _instance;
    }

    public Affector() {
        intakeMotor = new SparkMax(PincerMap.kIntakeMotorId.getDeviceId(), MotorType.kBrushless);

        setDefaultCommand(smartControlCommand(() -> ConditionalUitls.binaryToDirection(
            RobotContainer.operatorController.leftBumper().getAsBoolean(),
            RobotContainer.operatorController.rightBumper().getAsBoolean())));
    }

    public Command smartControlCommand(Supplier<Double> smartDirectionSupplier) {
        return run(
            () -> intakeMotor.set(
                RobotContainer.pose.classification == SuperPoseType.ALGAE ? 
                        smartDirectionSupplier.get() * AffectorConstants.kAlgaeDutyCycle : 
                        smartDirectionSupplier.get() * AffectorConstants.kCoralDutyCycle
            )
        ).withName("Affector Smart Control");
    }

    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return run(() -> {
            intakeMotor.set(dutyCycleSupplier.get());
        }).withName("Pincer Duty Cycle Command");
    }

    @Override
    public void periodic() {
        sendTelemetry();
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(getName() + "/Intake Position Rotations", intakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber(getName() + "/Intake Velocity Rotations", intakeMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber(getName() + "/Intake Tempurature", intakeMotor.getMotorTemperature());
    }

    @Override
    public void zeroSensors() {
        intakeMotor.getEncoder().setPosition(0);
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }
}