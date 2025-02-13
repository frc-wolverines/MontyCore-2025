package team5274.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import team5274.lib.hardware.drivers.DeviceId;
import team5274.robot.Constants.DriveConstants;
import team5274.robot.DeviceMap.DriveMap;

public class SwerveModule {
    public final int moduleNumber;

    private TalonFX driveMotor, pivotMotor;
    private CANcoder encoder;

    private PIDController controller;

    public SwerveModule(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        SwerveModuleDeviceMap map = DriveMap.driveMap[moduleNumber];
        SwerveModuleConfiguration config = DriveConstants.kModuleConfigs[moduleNumber];

        driveMotor = new TalonFX(map.driveId.getDeviceId());
        driveMotor.getConfigurator().apply(config.driveConfiguration);

        pivotMotor = new TalonFX(map.pivotId.getDeviceId());
        pivotMotor.getConfigurator().apply(config.pivotConfiguration);

        encoder = new CANcoder(map.encoderId.getDeviceId());

        controller = new PIDController(DriveConstants.kPivotP, DriveConstants.kPivotI, DriveConstants.kPivotD);
        controller.enableContinuousInput(-1, 1);
    }

    public int getNumber() {
        return moduleNumber;
    }

    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble() * DriveConstants.kPivotGearRatio;
    }

    public double getTrackPosition() {
        return driveMotor.getPosition().getValueAsDouble() * DriveConstants.kDriveGearRatio * DriveConstants.kTrackCircumference;
    }

    public double getTrackVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * DriveConstants.kDriveGearRatio * DriveConstants.kTrackCircumference;
    }

    public Rotation2d getPivotRot2d() {
        return Rotation2d.fromRotations(getPivotPosition());
    }

    public double getAbsPivotPosition() {
        return -encoder.getAbsolutePosition().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getTrackVelocity(), getPivotRot2d());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getTrackPosition(), getPivotRot2d());
    }

    public void setDesiredState(SwerveModuleState state) {
        
        //Discards small speeds
        if (state.speedMetersPerSecond < 0.001) {
            stop();
            return;
        }

        //Runs optimization algorithm to minimize distance traveled by the pivot motor
        state.optimize(getPivotRot2d());

        driveMotor.set(state.speedMetersPerSecond);

        pivotMotor.set(
            controller.calculate(getPivotPosition(), state.angle.getRotations())
        );
    }

    public void resetEncoders() {
        driveMotor.setPosition(0.0);
        pivotMotor.setPosition(0.0);
    }

    public void stop() {
        driveMotor.stopMotor();
        pivotMotor.stopMotor();
    }

    /**Zeros the pivot encoder's position to the current absolute encoders position*/
    public void zeroPivotPosition() {
        pivotMotor.setPosition(getAbsPivotPosition());
    }

    public static class SwerveModuleConfiguration {
        public final TalonFXConfiguration driveConfiguration, pivotConfiguration;

        public SwerveModuleConfiguration(TalonFXConfiguration driveConfig, TalonFXConfiguration pivotConfig) {
            this.driveConfiguration = driveConfig;
            this.pivotConfiguration = pivotConfig;
        }
    }

    public static class SwerveModuleDeviceMap {
        public final DeviceId driveId, pivotId, encoderId;
        
        public SwerveModuleDeviceMap(DeviceId driveId, DeviceId pivotId, DeviceId encoderId) {
            this.driveId = driveId;
            this.pivotId = pivotId;
            this.encoderId = encoderId;
        }
    }
}