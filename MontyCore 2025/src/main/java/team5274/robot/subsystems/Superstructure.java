package team5274.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team5274.robot.RobotContainer;
import team5274.robot.subsystems.elevator.Elevator;

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
        IDLE(0.65, 0.0, 4.33, 0.0),
        INTAKE_STATION(0.42, 1.0, 4.43, 0.0),
        HANG(0.0, 0.0, 4.22, 0.0),
        SCORE_TROUGH(0.32, 0.42, 5.07, 0.0),
        PREP_L1(0.44, 1.31, 4.44, Math.PI / 2), 
        SCORE_L1(0.44, 1.31, 4.8, Math.PI / 2), 
        PREP_L2(0.3, 2.7, 4.46, Math.PI / 2),
        SCORE_L2(0.3, 2.7, 4.9, Math.PI / 2),
        PREP_L3(0.08, 5.85, 4.98, Math.PI / 2),
        SCORE_L3(0.08, 5.85, 5.42, Math.PI / 2),
        ALGAE_L2(0.13, 2.53, 5.12, 0.0),
        ALGAE_L3(0.02, 4.75, 5.52, 0.0),

        DEBUG(0.0, 0.0, 4.33, 0.0),
        DEBUG_PLACE(0.0, 1.0, 4.33, 0.0),
        DEBUG_PLACE2(0.0, 1.8, 4.33, 0.0);


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

    public static Command pose(RobotContainer container, Supplier<SuperstructureGoal> goal) {
        Command[] commands = {
            container.elevatorPivot.angleCommand(goal.get().elevatorAngle),
            container.elevator.heightCommand(goal.get().elevatorHeight),
            container.arm.orientArm(goal.get().armAngle),
            container.arm.orientWrist(goal.get().wristAngle)
        };

        if(goal.get().elevatorHeight <= container.elevator.getHeight() || goal.get() == SuperstructureGoal.IDLE) Collections.reverse(Arrays.asList(commands)); //Checks if the elevator goal is lower than the current elevator height
        return new SequentialCommandGroup(commands).beforeStarting(() -> RobotContainer.currentGoal = goal.get()).withName("Pose to " + goal.get().name());
    }
    
    public static SuperstructureGoal interceptAndModifyGoal(SuperstructureGoal goal) {
        if(goal == SuperstructureGoal.PREP_L1 && RobotContainer.currentGoal == goal) goal = SuperstructureGoal.SCORE_L1;
        if(goal == SuperstructureGoal.PREP_L2 && RobotContainer.currentGoal == goal) goal = SuperstructureGoal.SCORE_L2;
        if(goal == SuperstructureGoal.PREP_L3 && RobotContainer.currentGoal == goal) goal = SuperstructureGoal.SCORE_L3;
        return goal;
    }
}
