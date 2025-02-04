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

        //Extrema
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 0.0; //Configure
        public static final double kMinHeight = 0.0;
        public static final double kMaxHeight = 0.0; //Configure
        public static final double kHomingHeight = kMinHeight;

        //Tolerances
        public static final double kRotationTolerance = 0.1;
        public static final double kHeightTolerance = 0.5;

        //Placement
        public static final double kTroughHeight = 0.0; //Configure
        public static final double kL1Height = 0.0; //Configure
        public static final double kL2Height = 0.0; //Configure
        public static final double kL3Height = 0.0; //Configure

        //Intaking
        public static final double kStationHeight = 0.0; //Configure
        public static final double kHandoffHeight = 0.0; //Configure

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
        public static final double kRotationTolerance = 0.1;

        //Intaking
        public static final double kStationAngle = 0.0; //Configure
        public static final double kHandoffAngle = 0.0; //Configure

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
}