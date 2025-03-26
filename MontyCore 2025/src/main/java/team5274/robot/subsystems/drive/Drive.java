package team5274.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import team5274.lib.util.ConditionalUitls;
import team5274.lib.util.LimelightHelpers;
import team5274.robot.Robot;
import team5274.robot.RobotContainer;
import team5274.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase implements SubsystemFrame {

    private List<SwerveModule> modules = new ArrayList<>();
    private AHRS gyroscope;
    
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field;

    private SlewRateLimiter xInputLimiter, yInputLimiter, rInputLimiter;
    private PIDController xAlignmentController = new PIDController(DriveConstants.kXAlignmentP, DriveConstants.kXAlignmentI, DriveConstants.kXAlignmentD);
    private PIDController yAlignmentController = new PIDController(DriveConstants.kYAlignmentP, DriveConstants.kYAlignmentI, DriveConstants.kYAlignmentD);

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

        SmartDashboard.putData("Robot/Field", field);

        xInputLimiter = new SlewRateLimiter(DriveConstants.kDriveMaxAcceleration);
        yInputLimiter = new SlewRateLimiter(DriveConstants.kDriveMaxAcceleration);
        rInputLimiter = new SlewRateLimiter(DriveConstants.kDriveMaxAngularAcceleration);

        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose2d,
                this::resetPose2d,
                this::getSpeeds,
                this::setSpeeds,
                new PPHolonomicDriveController(
                    new PIDConstants(0.0, 0.0, 0.0),
                    new PIDConstants(0.0, 0.0, 0.0)
                ),
                config,
                Robot::shouldFlipAuto,
                this
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        resetHeading();
        zeroSensors();

        setDefaultCommand(smartTeleOpControlCommand(
            () -> -RobotContainer.driverController.getLeftY(), 
            RobotContainer.driverController::getLeftX, 
            RobotContainer.driverController::getRightX,
            () -> ConditionalUitls.binaryToDirection(
                RobotContainer.driverController.rightBumper().getAsBoolean(), 
                RobotContainer.driverController.leftBumper().getAsBoolean()
            )
        ));
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        sendTelemetry();
    }

    /**
     * Constructs a command to control the Drive subsystem using per-axis input
     * @param xAxisSupplier a source of x-axis input (forward/backwards)
     * @param yAxisSupplier a source of y-axis input (left/right)
     * @param rAxisSupplier a source of r-axis input (clockwise/counter-clockwise)
     * @return A command which controls the Drive based on the axis input suppliers' values
     */
    public Command axisControlCommand(Supplier<Double> xAxisSupplier, Supplier<Double> yAxisSupplier, Supplier<Double> rAxisSupplier) {
        return run(() -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                -xInputLimiter.calculate(xAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -yInputLimiter.calculate(yAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -rInputLimiter.calculate(rAxisSupplier.get() * DriveConstants.kDriveMaxAllowedAngularSpeed)
            );

            setSpeeds(speeds);
        }); 
    }

    public Command smartTeleOpControlCommand(Supplier<Double> xAxisSupplier, Supplier<Double> yAxisSupplier, Supplier<Double> rAxisSupplier, Supplier<Double> branchSelectorSupplier) {
        return run(() -> {
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -yInputLimiter.calculate(xAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -xInputLimiter.calculate(yAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -rInputLimiter.calculate(rAxisSupplier.get() * DriveConstants.kDriveMaxAllowedAngularSpeed),
                getRotation2d()
            );

            if(branchSelectorSupplier.get() != 0 && LimelightHelpers.getTA("limelight") > DriveConstants.kAprilTagMinArea) {
                double tx = branchSelectorSupplier.get() == -1 ? DriveConstants.kLeftBranchTx : DriveConstants.kRightBranchTx;
                double ta = branchSelectorSupplier.get() == -1 ? DriveConstants.kApriTagTargetAreaLeft : DriveConstants.kAprilTagTargetAreaRight;

                System.out.println(xAlignmentController.calculate(LimelightHelpers.getTX("limelight"), tx));

                speeds = new ChassisSpeeds(
                    -xAlignmentController.calculate(LimelightHelpers.getTA("limelight"), ta),
                    yAlignmentController.calculate(LimelightHelpers.getTX("limelight"), tx),
                    0.0
                );
            }

            setSpeeds(speeds);
        });
    }

    /**
     * Constructs a command to control the Drive subsystem using per-axis input with field-centric offsets
     * @param xAxisSupplier a source of x-axis input (forward/backwards)
     * @param yAxisSupplier a source of y-axis input (left/right)
     * @param rAxisSupplier a source of r-axis input (clockwise/counter-clockwise)
     * @return A command which controls the Drive based on the axis input suppliers' values
     */
    public Command fieldAxisControlCommand(Supplier<Double> xAxisSupplier, Supplier<Double> yAxisSupplier, Supplier<Double> rAxisSupplier) {
        return run(() -> {
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                -yInputLimiter.calculate(xAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -xInputLimiter.calculate(yAxisSupplier.get() * DriveConstants.kDriveMaxAllowedSpeed),
                -rInputLimiter.calculate(rAxisSupplier.get() * DriveConstants.kDriveMaxAllowedAngularSpeed),
                getRotation2d()
            );

            setSpeeds(speeds);
        }); 
    }

    /**
     * Retrieves an array of SwerveModulePositions from the SwerveModules
     * @return an array of SwerveModulePositions pulled from the Swerve Modules' instances
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            modules.get(0).getPosition(),
            modules.get(1).getPosition(),
            modules.get(2).getPosition(),
            modules.get(3).getPosition()
        };
    }

    /**
     * Retrieves an array of SwerveModuleStates from the Swerve Modules
     * @return an array of SwerveModuleStates pulled from the Swerve Modules' instances
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            modules.get(0).getState(),
            modules.get(1).getState(),
            modules.get(2).getState(),
            modules.get(3).getState()
        };
    }

    /**
     * Retrieves the estimated Pose2d given from the SwerveDrivePoseEstimator
     * @return the robot's Pose2d
     */
    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Retrieves the ChassisSpeeds given from the SwerveDriveKinematics based off of the SwerveModuleStates of the modules
     * @return the robot's ChassisSpeeds
     */
    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Retrieves the current heading of the robot given from the AHRS (NavX2) gyroscope
     * @return the robot's heading in degrees
     */
    public double getHeading() {
        return Math.IEEEremainder(gyroscope.getAngle(), 360) + 180;
    }

    /**
     * Retrieves the current Rotation2d given from the AHRS (NavX2) gyroscope
     * @return the robot's Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Sets the desired speed of the robot
     * @param speeds a ChassisSpeeds object representing the desired speeds of the robot
     */
    public void setSpeeds(ChassisSpeeds speeds) {

        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kDriveMaxAllowedSpeed);

        modules.forEach((module) -> {
            module.setDesiredState(states[module.getNumber()]);
        });
    }

    @Override
    public void sendTelemetry() {
        SmartDashboard.putData(this);
    }

    @Override
    public void zeroSensors() {
        modules.forEach((module) -> {module.zeroPivotPosition(); module.resetEncoders();});
    }

    public Command zero() {
        return runOnce(() -> {zeroSensors();});
    }

    @Override
    public void stop() {
        modules.forEach((module) -> {module.stop();});
    }
    
    public void resetHeading() {
        gyroscope.reset();
    }

    public Command reset() {
        return runOnce(() -> {
            resetHeading();
        });
    }
    
    public void resetPose2d(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }
}
