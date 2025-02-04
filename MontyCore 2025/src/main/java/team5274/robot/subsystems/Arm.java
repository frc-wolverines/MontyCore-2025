package team5274.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.DeviceMap.ArmMap;

public class Arm extends SubsystemBase implements SubsystemFrame {
    private TalonFX pinionMotor;
    DutyCycleEncoder encoder;

    private static Arm _instance;

    public static Arm get() {
        if(_instance == null) _instance = new Arm();
        return _instance;
    }

    public Arm() {
        pinionMotor = new TalonFX(0);
        encoder = new DutyCycleEncoder(new DigitalInput(ArmMap.kEncoderId.getDeviceId()));
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);

        SmartDashboard.putNumber(this.getName() + "/Pinion Position Rotations", pinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(this.getName() + "/Pinion Velocity Rotations", pinionMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putData(this.getName() + "/Through Bore Encoder", encoder);
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