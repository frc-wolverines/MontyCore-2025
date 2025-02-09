package team5274.robot.subsystems.arm;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.PincerConstants;
import team5274.robot.DeviceMap.PincerMap;

public class Pincer extends SubsystemBase implements SubsystemFrame {
    private TalonFX wristMotor;
    private SparkMax intakeMotor;
    private ColorSensorV3 colorSensor;

    private static Pincer _instance;

    public static Pincer get() {
        if(_instance == null) _instance = new Pincer();
        return _instance;
    }

    public Pincer() {
        wristMotor = new TalonFX(PincerMap.kWristMotorId.getDeviceId());
        wristMotor.getConfigurator().apply(PincerConstants.kWristConfig);

        intakeMotor = new SparkMax(PincerMap.kIntakeMotorId.getDeviceId(), MotorType.kBrushless);

        colorSensor = new ColorSensorV3(Port.kMXP);

        setDefaultCommand(setupNeutral().alongWith(dutyCycleIntakeCommand(() -> 0.0)));
    }

    public boolean hasGamepiece() {
        return colorSensor.getProximity() >= PincerConstants.kColorSensorPossesionProximityThreshold;
    }

    public Command smartRunIntake() {
        return hasGamepiece() ? dutyCycleIntakeCommand(() -> PincerConstants.kDepositDutyCycle) : runUntilSuccessfulIntakeCommand();
    }

    public Command dutyCycleIntakeCommand(Supplier<Double> dutyCycleSupplier) {
        return runEnd(
            () -> {
                intakeMotor.set(dutyCycleSupplier.get());
                if(dutyCycleSupplier.get() > 0) {
                    RobotContainer.operatorController.setRumble(RumbleType.kLeftRumble, dutyCycleSupplier.get() * 0.5);
                } else if (dutyCycleSupplier.get() < 0) {
                    RobotContainer.operatorController.setRumble(RumbleType.kRightRumble, dutyCycleSupplier.get() * 0.5);
                } else {
                    RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0);
                }
            },
            () -> intakeMotor.stopMotor()
        ).withName("Pincer Intake duty cycle");
    }

    public Command runUntilSuccessfulIntakeCommand() {
        return runEnd(
            () -> intakeMotor.set(PincerConstants.kIntakeDutyCycle), 
            () -> intakeMotor.stopMotor()
        ).unless(this::hasGamepiece).withName("Pincer Intake Until Gamepiece");
    }

    public Command dutyCycleWristCommand(Supplier<Double> dutyCycleSupplier) {
        return runEnd(
            () -> wristMotor.setControl(new DutyCycleOut(dutyCycleSupplier.get())),
            () -> wristMotor.setControl(new NeutralOut())
        ).withName("Pincer Wrist duty cycle");
    }

    public Command motionMagicWristCommand(Supplier<Double> motionMagicPositionSupplier) {
        return runEnd(
            () -> wristMotor.setControl(new MotionMagicDutyCycle(motionMagicPositionSupplier.get())),
            () -> wristMotor.setControl(new NeutralOut())
        ).withName("Pincer Wrist MotionMagic");
    }

    public Command setupNeutral() {
        return motionMagicWristCommand(() -> PincerConstants.kWristNeutralPosition);
    }

    public Command setupCoral() {
        return motionMagicWristCommand(() -> PincerConstants.kWristVerticalCoralPosition);
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(getName() + "/Wrist Position Rotations", wristMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(getName() + "/Wrist Velocity Rotations", wristMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber(getName() + "/Intake Position Rotations", intakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber(getName() + "/Intake Velocity Rotations", intakeMotor.getEncoder().getVelocity());

        SmartDashboard.putNumber(getName() + "/Color Sensor Proximity", colorSensor.getProximity());
        SmartDashboard.putString(getName() + "/Color Sensor Color", "" + colorSensor.getRed() + ", " + (colorSensor.getGreen() / 2) + ", " + colorSensor.getBlue());
    }

    @Override
    public void zeroSensors() {
        wristMotor.setPosition(0);
        intakeMotor.getEncoder().setPosition(0);
    }

    @Override
    public void stop() {
        wristMotor.stopMotor();
        intakeMotor.stopMotor();
    }
}