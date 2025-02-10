package team5274.robot.subsystems.arm;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.Constants.PincerConstants;
import team5274.robot.DeviceMap.PincerMap;

public class Pincer extends SubsystemBase implements SubsystemFrame {
    private SparkMax intakeMotor;
    private ColorSensorV3 colorSensor;

    private static Pincer _instance;

    public static Pincer get() {
        if(_instance == null) _instance = new Pincer();
        return _instance;
    }

    public Pincer() {
        intakeMotor = new SparkMax(PincerMap.kIntakeMotorId.getDeviceId(), MotorType.kBrushless);
        colorSensor = new ColorSensorV3(Port.kMXP);
    }

    public boolean hasGamepiece() {
        return colorSensor.getProximity() >= PincerConstants.kColorSensorPossesionProximityThreshold;
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

        SmartDashboard.putNumber(getName() + "/Color Sensor Proximity", colorSensor.getProximity());
        SmartDashboard.putString(getName() + "/Color Sensor Color", "" + colorSensor.getRed() + ", " + (colorSensor.getGreen() / 2) + ", " + colorSensor.getBlue());
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