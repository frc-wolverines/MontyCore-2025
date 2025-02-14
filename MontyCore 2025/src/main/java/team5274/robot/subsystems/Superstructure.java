package team5274.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    public enum SuperstructureGoal {
        IDLE(0.0, 0.0, 0.0, 0.0),
        INTAKE_STATION(0.0, 0.0, 0.0, 0.0),
        HANG(0.0, 0.0, 0.0, 0.0),
        SCORE_TROUGH(0.0, 0.0, 0.0, 0.0),
        SCORE_L1(0.36, 4.0, 0.0, Math.PI / 4),
        SCORE_L2(0.0, 0.0, 0.0, Math.PI / 4),
        SCORE_L3(0.0, 0.0, 0.0, Math.PI / 4);

        public final double elevatorAngle;
        public final double elevatorHeight;
        public final double armAngle;
        public final double wristAngle;

        private SuperstructureGoal(double elevator_angle, double elevator_height, double arm_angle, double wrist_angle) {
            this.elevatorAngle = elevator_angle;
            this.elevatorHeight = elevator_height;
            this.armAngle = arm_angle;
            this.wristAngle = wrist_angle;
        }
    }

    public static Command pose(RobotContainer container, SuperstructureGoal goal) {
        return new ParallelCommandGroup(
            container.elevatorPivot.angleCommand(goal.elevatorAngle),
            container.elevator.heightCommand(goal.elevatorHeight)
        );
    }
}
