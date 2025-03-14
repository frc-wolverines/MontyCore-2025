package team5274.robot.subsystems;

import static edu.wpi.first.units.Units.Newton;

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
import team5274.robot.Robot;
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

        private SuperstructureGoal(double elevator_angle, double elevator_height, double arm_angle, double wrist_angle) {
            this.elevatorAngle = elevator_angle;
            this.elevatorHeight = elevator_height;
            this.armAngle = arm_angle;
            this.wristAngle = wrist_angle;
        }
    }

    public static Command pose(RobotContainer container, Supplier<SuperstructureGoal> goal) {
        // Command[] commands = {
            // container.elevatorPivot.angleCommand(goal.get().elevatorAngle),
            // container.elevator.heightCommand(goal.get().elevatorHeight),
            // container.arm.orientArm(goal.get().armAngle),
            // container.arm.orientWrist(goal.get().wristAngle)
        // };

        Command[] commands = {};
        Command[] normal = {
            Commands.idle(container.drive).withTimeout(1).withName("Elevator Rotates Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Elevator Moves Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Arm Rotates Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Wrist Rotates Now")
        };
        Command[] inverse = {
            Commands.idle(container.drive).withTimeout(1).withName("Wrist Rotates Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Arm Rotates Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Elevator Moves Now"),
            Commands.idle(container.drive).withTimeout(1).withName("Elevator Rotates Now")
        };

         //Checks if the elevator goal is lower than the current elevator height
        //  if(goal.get().elevatorHeight < RobotContainer.getCurrentGoal().elevatorHeight || goal.get() == SuperstructureGoal.IDLE) Collections.reverse(Arrays.asList(commands));
        return new SequentialCommandGroup(commands).beforeStarting(() -> {
            for(Command command : commands) {
                System.out.println(command.getName());
            }
        }).beforeStarting(() -> RobotContainer.currentGoal = goal.get()).beforeStarting(() -> {
            System.out.println("");
            System.out.println(isRetracting(RobotContainer::getCurrentGoal, goal) + "   " + RobotContainer.getCurrentGoal() + " -> " + goal.get());
            if(isRetracting(RobotContainer::getCurrentGoal, goal)) commands = inverse;
        });
    }

    public static boolean isRetracting(Supplier<SuperstructureGoal> currentGoal, Supplier<SuperstructureGoal> newGoal) {
        boolean elevatorRetracting = newGoal.get().elevatorHeight < currentGoal.get().elevatorHeight;
        boolean vetoExtending = newGoal.get() == SuperstructureGoal.IDLE;

        return elevatorRetracting || vetoExtending;
    }
}
