// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team5274.robot.subsystems.drive.SwerveModule.SwerveModuleConfiguration;

/** Add your docs here. */
public class Constants {

    public static class DriveConstants {

        public static final double kWheelDistance = Units.inchesToMeters(23.75);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelDistance / 2, -kWheelDistance / 2),
            new Translation2d(kWheelDistance / 2, kWheelDistance / 2),
            new Translation2d(-kWheelDistance / 2, -kWheelDistance / 2),
            new Translation2d(-kWheelDistance / 2, kWheelDistance / 2)
        );

        public static final double kPivotP = 0.8;
        public static final double kPivotI = 0.0;
        public static final double kPivotD = 0.0;

        public static final double kPivotGearRatio = 1 / 18.75; //Configure
        public static final double kDriveGearRatio = 1 / 5.36; //Configure

        public static final double kTrackCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final double kDriveMaxAcceleration = 6.0; //Configure
        public static final double kDriveMaxAngularAcceleration = 6.0; //Configure

        public static final double kDriveMaxAllowedSpeed = 1.0; //Configure
        public static final double kDriveMaxAllowedAngularSpeed = 1.0; //Configure

        public static final double kXP = 0.25;
        public static final double kYP = 0.25;
        public static final double kRP = 0.25;

        public static final TalonFXConfiguration kDriveMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        public static final TalonFXConfiguration kDriveInvertedMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        public static final TalonFXConfiguration kPivotMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));

        //LF, RF, LB, RB
        public static final SwerveModuleConfiguration[] kModuleConfigs = {
            new SwerveModuleConfiguration(kDriveInvertedMotorConfig, kPivotMotorConfig),
            new SwerveModuleConfiguration(kDriveMotorConfig, kPivotMotorConfig),
            new SwerveModuleConfiguration(kDriveInvertedMotorConfig, kPivotMotorConfig),
            new SwerveModuleConfiguration(kDriveMotorConfig, kPivotMotorConfig)
        };
    }

    public static class ElevatorConstants {
        public static final double kGearRatio = 1 / 25.0; //Configure
        public static final double kHeightRotationRatio = 1 / 25.0; //Configure

        //Extrema
        public static final double kMinRotations = 0.0;
        public static final double kMaxRotations = 0.0; //Configure
        public static final double kMinHeight = 0.0;
        public static final double kMaxHeight = 0.0; //Configure
        public static final double kHomingHeight = kMinHeight;

        //Tolerances
        public static final double kRotationTolerance = 0.1;
        public static final double kHeightTolerance = 0.1;

        public static final double kP = 0.6;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

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
        public static final double kP = 6.5; 
        public static final double kI = 0.0; 
        public static final double kD = 0.0; 

        public static final double kPP = 3.5; 
        public static final double kPI = 0.0; 
        public static final double kPD = 0.0; 

        //Hardware Configurations
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));

        public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
    }

    public static class ArmConstants {
        public static final double kWristGearRatio = 1 / 25; //Configure

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
        public static final double kWristAngleTolerance = 0.2; //Configure

        //PID
        public static final double kArmP = 0.75; //Configure
        public static final double kArmI = 0.0; //Configure
        public static final double kArmD = 0.0; //Configure


        public static final double kWristP = 0.08; //Configure
        public static final double kWristI = 0.0; //Configure
        public static final double kWristD = 0.0; //Configure


        //Hardware Configurations
        public static final TalonFXConfiguration kPinionConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(20)
                .withStatorCurrentLimitEnable(true));

        public static final TalonFXConfiguration kWristConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
    }

    public static class PincerConstants {
        //Tolerances
        public static final double kColorSensorPossesionProximityThreshold = 0.0; //Configure

        //Setpoints
        public static final double kIntakeDutyCycle = 1.0;
        public static final double kDepositDutyCycle = -1.0;
    }
}