package team5274.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.robot.RobotContainer;
import team5274.robot.DeviceMap.ClimberClampMap;
import team5274.robot.subsystems.elevator.Elevator;

public class ClimberClamp extends SubsystemBase {
    private static ClimberClamp _instance;
    private final TalonFX clampMotor;
    // private final PIDController contoller;

    public static ClimberClamp get() {
        if(_instance == null) _instance = new ClimberClamp();
        return _instance;
    }

    public ClimberClamp() {
        clampMotor = new TalonFX(ClimberClampMap.clampMotorId.getDeviceId());

        setDefaultCommand(dutyCycleCommand(
            () -> RobotContainer.driverController.getRightTriggerAxis() * 0.25 - RobotContainer.driverController.getLeftTriggerAxis() * 0.25
        ));
    }

    public Command dutyCycleCommand(Supplier<Double> dutyCycleSupplier) {
        return run(() -> clampMotor.setControl(new DutyCycleOut(dutyCycleSupplier.get())));
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber(getName() + "/Clamp Motor Position", clampMotor.getPosition().getValueAsDouble());
    }
}
