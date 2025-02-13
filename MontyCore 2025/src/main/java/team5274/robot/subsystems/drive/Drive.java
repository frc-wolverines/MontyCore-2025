package team5274.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team5274.lib.control.SubsystemFrame;
import team5274.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase implements SubsystemFrame {

    private List<SwerveModule> modules = new ArrayList<>();
    private AHRS gyroscope;
    
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;

    public static Drive _instance;

    public static Drive get() {
        if(_instance == null) _instance = new Drive();
        return _instance;
    }

    public Drive() {
        modules.add(new SwerveModule(0));
        modules.add(new SwerveModule(1));
        modules.add(new SwerveModule(2));
        modules.add(new SwerveModule(3));

        gyroscope = new AHRS(NavXComType.kMXP_SPI);

        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions(), new Pose2d());
        field = new Field2d();

        SmartDashboard.putData(field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        sendTelemetry();
    }

    public Command axisControlCommand(Supplier<Double> xAxisSupplier, Supplier<Double> yAxisSupplier, Supplier<Double> rAxisSupplier) {
        return run(() -> {
            
        });
    }

    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            modules.get(0).getPosition(),
            modules.get(1).getPosition(),
            modules.get(2).getPosition(),
            modules.get(3).getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            modules.get(0).getState(),
            modules.get(1).getState(),
            modules.get(2).getState(),
            modules.get(3).getState()
        };
    }

    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose2d(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetHeading() {
        gyroscope.reset();
    }

    public void setSpeeds(ChassisSpeeds speeds) {

        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0);

        modules.forEach((module) -> {
            module.setDesiredState(states[module.getNumber()]);
        });
    }

    public void setRelativeSpeeds(ChassisSpeeds speeds) {
        setSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);
        SmartDashboard.putNumber(getName() + "/Heading", getHeading());

        modules.forEach((module) -> {
            SmartDashboard.putNumber(getName() + "/Module " + module.getNumber() + "/Absolute Position", module.getAbsPivotPosition());
            SmartDashboard.putNumber(getName() + "/Module " + module.getNumber() + "/Pivot Position", module.getPivotPosition());
            SmartDashboard.putNumber(getName() + "/Module " + module.getNumber() + "/Track Position", module.getTrackPosition());
            SmartDashboard.putNumber(getName() + "/Module " + module.getNumber() + "/Track Velocity", module.getTrackVelocity());
        });
    }

    @Override
    public void zeroSensors() {
        modules.forEach((module) -> {module.resetEncoders(); module.zeroPivotPosition();});
    }

    @Override
    public void stop() {
        modules.forEach((module) -> {module.stop();});
    }
}
