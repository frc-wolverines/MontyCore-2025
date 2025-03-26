package team5274.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team5274.robot.RobotContainer;

public class Superstructure {
    public enum SuperPoseType {
        NEUTRAL,
        CORAL,
        ALGAE
    }

    /** 
     * A enum representation of the robot's goal. An instance of this can be used by multiple mechanisms for position data.
     * <p> A SuperstructureGoal stores: </p>
     * <ul>
     * <li> Elevator angle (in degrees relative to the verticality of the pivot, a positive angle is leaning closer to the front of the robot) </li>
     * <li> Elevator height (in inches relative to the collapsed starting position of the carriage) </li>
     * <li> Arm angle (in degrees relative to the maximum position of the arm, a positive angle is rotated so that the claw is facing downwards) </li>
     * <li> Wrist angle (in degrees relative to the end-affector being horizontal) </li>
     * </ul>
    */
    public enum SuperPose {
        IDLE(0.64, 0.0, 4.33, 0.0, SuperPoseType.NEUTRAL),
        INTAKE_STATION(0.48, 0.55, 4.33, 0.0, SuperPoseType.CORAL),
        SCORE_TROUGH(0.33, 0.0, 4.9, 0.0, SuperPoseType.CORAL),
        PREP_L1(0.33, 0.79, 4.49, Math.PI / 2, SuperPoseType.CORAL), 
        SCORE_L1(0.33, 0.79, 4.98, Math.PI / 2, SuperPoseType.CORAL), 
        PREP_L2(0.28, 1.63, 4.5, Math.PI / 2, SuperPoseType.CORAL),
        SCORE_L2(0.28, 1.63, 4.97, Math.PI / 2, SuperPoseType.CORAL),
        PREP_L3(0.065, 3.5, 4.85, Math.PI / 2, SuperPoseType.CORAL),
        SCORE_L3(0.065, 3.5, 5.45, Math.PI / 2, SuperPoseType.CORAL),

        ALGAE_L2(0.46, 1.04, 4.32, 0.0, SuperPoseType.ALGAE),
        ALGAE_L3(0.31, 1.83, 4.35, 0.0, SuperPoseType.ALGAE),

        DEBUG(0.0, 0.0, 4.33, 0.0, SuperPoseType.NEUTRAL),
        DEBUG_PLACE(0.0, 1.0, 4.33, 0.0, SuperPoseType.NEUTRAL),
        DEBUG_PLACE2(0.0, 1.8, 4.33, 0.0, SuperPoseType.NEUTRAL);


        public final double elevatorAngle;
        public final double elevatorHeight;
        public final double armAngle;
        public final double wristAngle;
        public final SuperPoseType classification;

        private SuperPose(double elevator_angle, double elevator_height, double arm_angle, double wrist_angle, SuperPoseType classification) {
            this.elevatorAngle = elevator_angle;
            this.elevatorHeight = elevator_height;
            this.armAngle = arm_angle;
            this.wristAngle = wrist_angle;
            this.classification = classification;
        }
    }

    public static class SuperConstants {

        public static final double kArmSafeCoralPosition = 0.0;
        public static final double kArmSafeAlgaePosition = 0.0;

        public static final double kArmSafeWristPosition = 0.0;

        public static final double kElevatorMinUnsafePosition = 0.0;
        public static final double kElevatorMaxUnsafePosition = 0.0;

    }

    public static class SuperCommandFactory {
        public static Command build(RobotContainer container, SuperPose pose) {
            return Commands.runOnce(() -> {
                Command[] sequence = {
                    container.elevatorPivot.angleCommand(pose.elevatorAngle),
                    container.elevator.heightCommand(pose.elevatorHeight),
                    container.arm.orientArm(pose.armAngle),
                    container.arm.orientWrist(pose.wristAngle)
                };

                if(pose.elevatorHeight <= RobotContainer.pose.elevatorHeight) Collections.reverse(Arrays.asList(sequence));
                Commands.sequence(sequence).beforeStarting(() -> RobotContainer.pose = pose).withName("Pose to " + pose.name()).schedule();
            });
        }
    }
}