// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team5274.robot.DeviceMap;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.DeviceMap.ElevatorMap;
import team5274.lib.control.SubsystemFrame;

/** Add your docs here. */
public class Elevator implements SubsystemFrame {
    public enum ControlState {
        DUTY_CYCLE,
        OBJECTIVE
    }

    private ControlState controlState = ControlState.OBJECTIVE;
    private double heightObjective = ElevatorConstants.kMinHeight;
    private TalonFX master, slave;

    public static Elevator _instance;

    public static Elevator get() {
        if(_instance == null) _instance = new Elevator();
        return _instance;
    }

    public Elevator() {
        master = new TalonFX(ElevatorMap.kMasterMotorId.getDeviceId());
        master.getConfigurator().apply(ElevatorConstants.kMasterConfig);

        slave = new TalonFX(ElevatorMap.kSlaveMotorId.getDeviceId());
        slave.getConfigurator().apply(ElevatorConstants.kSlaveConfig);
    }

    public void writeDutyCycleOutput(double power) {
        if(controlState != ControlState.DUTY_CYCLE) controlState = ControlState.DUTY_CYCLE;
        master.set(power);
        slave.setControl(new Follower(master.getDeviceID(), true));
    } 

    @Override
    public void sendTelemetry() {
        SmartDashboard.putNumber("Master Position Rotations", master.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Master Velocity Rotations", master.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Slave Position Rotations", slave.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Slave Velocity Rotations", slave.getVelocity().getValueAsDouble());
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