package team5274.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team5274.robot.RobotContainer;

public class Superstructure {

    /** 
     * A enum representation of the robot's goal. An instance of this can be used by multiple mechanisms for position data.
     * <p> A SuperstructureGoal stores: </p>
     * <ul>
     * <li> Arm angle (in degrees relative to the minimum position of the arm, a positive angle is rotated so that the claw is facing upwards) </li>
     * <li> Elevator height (in inches relative to the collapsed starting position of the carriage) </li>
     * <li> Elevator angle (in degrees relative to the verticality of the pivot, a positive angle is leaning closer to the front of the robot) </li>
     * </ul>
    */
    public enum SuperstructureGoal {
        IDLE(0.0, 0.0, 0.0, 0.0),
        INTAKE_STATION(0.0, 0.0, 0.0, 0.0),
        HANG(0.0, 0.0, 0.0, 0.0),
        SCORE_TROUGH(0.0, 0.0, 0.0, 0.0),
        SCORE_L1(0.0, 0.0, 0.0, Math.PI / 4),
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
}
