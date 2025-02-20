package team5274.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import team5274.robot.RobotContainer;

public class Path {
    public static AutoRoutine testMoveForward(RobotContainer container) {
        AutoRoutine routine = container.autoFactory.newRoutine("testMoveForward");

        AutoTrajectory testForward = routine.trajectory("TestForward");

        routine.active().onTrue(
            Commands.sequence(
                testForward.resetOdometry(),
                testForward.cmd()
            )
        );

        return routine;
    }
}
