package team5274.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team5274.robot.RobotContainer;

public class Superstructure {

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
    public enum SuperstructurePose {
        IDLE(0.64, 0.0, 4.33, 0.0),
        INTAKE_STATION(0.51, 0.55, 4.33, 0.0),
        HANG(0.0, 0.0, 4.22, 0.0),
        SCORE_TROUGH(0.33, 0.0, 4.9, 0.0),
        PREP_L1(0.33, 0.79, 4.49, Math.PI / 2), 
        SCORE_L1(0.33, 0.79, 4.98, Math.PI / 2), 
        PREP_L2(0.28, 1.63, 4.5, Math.PI / 2),
        SCORE_L2(0.28, 1.63, 4.97, Math.PI / 2),
        PREP_L3(0.065, 3.5, 4.85, Math.PI / 2),
        SCORE_L3(0.065, 3.5, 5.45, Math.PI / 2),
        ALGAE_L2(0.46, 1.04, 4.32, 0.0),
        ALGAE_L3(0.31, 1.83, 4.35, 0.0),

        DEBUG(0.0, 0.0, 4.33, 0.0),
        DEBUG_PLACE(0.0, 1.0, 4.33, 0.0),
        DEBUG_PLACE2(0.0, 1.8, 4.33, 0.0);


        public final double elevatorAngle;
        public final double elevatorHeight;
        public final double armAngle;
        public final double wristAngle;

        private SuperstructurePose(double elevator_angle, double elevator_height, double arm_angle, double wrist_angle) {
            this.elevatorAngle = elevator_angle;
            this.elevatorHeight = elevator_height;
            this.armAngle = arm_angle;
            this.wristAngle = wrist_angle;
        }
    }

    public static Command pose(RobotContainer container, SuperstructurePose pose) {
        // List<Command> poseSequence = new ArrayList<Command>(Arrays.asList(
        //     Commands.none().withTimeout(1).withName("Elevator Pivots Now"),
        //     Commands.none().withTimeout(1).withName("Elevator Extends/Collapses Now"),
        //     Commands.none().withTimeout(1).withName("Arm Pivots Now"),
        //     Commands.none().withTimeout(1).withName("Wrist Rotates Now")
        // ));

        List<Command> poseSequence = new ArrayList<Command>(Arrays.asList(
            container.elevatorPivot.angleCommand(pose.elevatorAngle),
            container.elevator.heightCommand(pose.elevatorHeight),
            container.arm.orientArm(pose.armAngle),
            container.arm.orientWrist(pose.wristAngle)
        ));

        return Commands.sequence(poseSequence.toArray(new Command[4]))
            .beforeStarting(() -> {
                RobotContainer._robotPose = pose; 
            })
            .beforeStarting(() -> { 
                if(!willCollapse(pose) && !isOriginal(poseSequence)) Collections.reverse(poseSequence);
                if(willCollapse(pose) && isOriginal(poseSequence)) Collections.reverse(poseSequence);
                System.out.println("\n");
                System.out.println("Will Collapse: " + willCollapse(pose));
                System.out.println("Change: " + RobotContainer._robotPose + " -> " + pose);
                System.out.println("Order:");
                for(Command command : poseSequence) System.out.println("  " + command.getName());
            })
            .withName("Pose to " + pose);
    }

    public static boolean isOriginal(List<Command> sequence) {
        return sequence.get(0).getName() == "Elevator Pivots Now";
    }

    public static boolean willCollapse(SuperstructurePose newPose) {
        return newPose.elevatorHeight < RobotContainer._robotPose.elevatorHeight;
    }
}