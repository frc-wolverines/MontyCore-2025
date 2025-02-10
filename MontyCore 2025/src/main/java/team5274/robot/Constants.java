// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class Constants {

    public static class ElevatorConstants {
        public static final double kGearRatio = 0.0; //Configure
        public static final double kHeightRotationRatio = 0.0; //Configure

        //Extrema
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 0.0; //Configure
        public static final double kMinHeight = 0.0;
        public static final double kMaxHeight = 0.0; //Configure
        public static final double kHomingHeight = kMinHeight;

        //Tolerances
        public static final double kRotationTolerance = 0.1;
        public static final double kHeightTolerance = 0.5;

        //Motion Magic®
        public static final double kMotionMagicAcceleration = 20;
        public static final double kMotionMagicCruiseVelocity = 10;

        //Hardware Configurations
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(kMotionMagicAcceleration)
                    .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity));

        public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(kMotionMagicAcceleration)
                    .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity));
    }

    public static class ElevatorPivotConstants {
        public static final double kGearRatio = 0.0; //Configure

        //Extrema
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 0.0; //Configure
        public static final double kMinAngle = 0.0;
        public static final double kMaxAngle = 0.0; //Configure

        //Tolerances
        public static final double kAngleTolerance = 0.05;

        //PID
        public static final double kP = 0.0; //Configure
        public static final double kI = 0.0; //Configure
        public static final double kD = 0.0; //Configure

        //Hardware Configurations
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

        public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
    }

    public static class ArmConstants {
        public static final double kArmGearRatio = 0.0; //Configure
        public static final double kWristGearRatio = 0.0; //Configure

        //Extrema
        public static final double kArmMinRotations = 0.0;
        public static final double kArmMaxRotations = 0.0; //Configure
        public static final double kArmMinAngle = 0.0;
        public static final double kArmMaxAngle = 0.0; //Configure

        public static final double kWristMinRotations = 0.0;
        public static final double kWristMaxRotations = 0.0; //Configure
        public static final double kWristMinAngle = 0.0;
        public static final double kWristMaxAngle = 0.0; //Configure

        //Tolerances
        public static final double kArmAngleTolerance = 0.05; //Configure
        public static final double kWristAngleTolerance = 0.05; //Configure

        //Motion Magic®
        public static final double kWristMotionMagicAcceleration = 20; //Configure
        public static final double kWristMotionMagicCruiseVelocity = 10; //Configure

        //PID
        public static final double kP = 0.0; //Configure
        public static final double kI = 0.0; //Configure
        public static final double kD = 0.0; //Configure

        //Hardware Configurations
        public static final TalonFXConfiguration kPinionConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

        public static final TalonFXConfiguration kWristConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(kWristMotionMagicAcceleration)
            .withMotionMagicCruiseVelocity(kWristMotionMagicCruiseVelocity));
    }

    public static class PincerConstants {
        //Tolerances
        public static final double kColorSensorPossesionProximityThreshold = 0.0; //Configure

        //Setpoints
        public static final double kIntakeDutyCycle = 1.0;
        public static final double kDepositDutyCycle = -1.0;
    }
}