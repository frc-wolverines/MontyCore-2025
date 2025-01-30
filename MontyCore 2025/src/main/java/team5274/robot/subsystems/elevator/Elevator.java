// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import team5274.robot.DeviceMap;
import team5274.robot.Constants.ElevatorConstants;
import team5274.robot.DeviceMap.ElevatorMap;
import team5274.lib.control.SubsystemFrame;

/** Add your docs here. */
public class Elevator implements SubsystemFrame {
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
        slave.getConfigurator().apply(ElevatorConstants.kSlaveConfig)
    }
}