// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5274.robot.auto.Path;
import team5274.robot.subsystems.Superstructure;
import team5274.robot.subsystems.Superstructure.SuperstructureGoal;
import team5274.robot.subsystems.arm.Arm;
import team5274.robot.subsystems.arm.Pincer;
import team5274.robot.subsystems.drive.Drive;
import team5274.robot.subsystems.elevator.Elevator;
import team5274.robot.subsystems.elevator.ElevatorPivot;

public class RobotContainer {

  public final static CommandXboxController driverController = new CommandXboxController(0);
  public final static CommandXboxController operatorController = new CommandXboxController(1);

  public final AutoFactory autoFactory;

  public Drive drive = Drive.get();
  // public Arm arm = Arm.get();
  // public Pincer pincer = Pincer.get();

  public RobotContainer() {
    autoFactory = new AutoFactory(
      drive::getPose2d,
      drive::resetPose2d,
      drive::followTrajectory,
      true,
      drive
    );
    configureBindings();
  }

  private void configureBindings() {

    // driverController.a().toggleOnTrue(Superstructure.pose(this, SuperstructureGoal.IDLE));
    // driverController.b().toggleOnTrue(Superstructure.pose(this, SuperstructureGoal.SCORE_L1));

    // driverController.y().onTrue(drive.zero());
    driverController.start().onTrue(drive.reset());

    // driverController.a().toggleOnTrue(arm.angleCommand(SuperstructureGoal.SCORE_L1.armAngle));

    // operatorController.a().toggleOnTrue(Superstructure.pose(this, SuperstructureGoal.IDLE));
    // operatorController.b().toggleOnTrue(Superstructure.pose(this, SuperstructureGoal.SCORE_L1));

    // operatorController.x().toggleOnTrue(arm.angleCommand(SuperstructureGoal.SCORE_L1.armAngle, Math.PI / 2));
    // operatorController.y().toggleOnTrue(arm.angleCommand(SuperstructureGoal.SCORE_L1.armAngle, 0));

    // driverController.a().toggleOnTrue(elevator.heightCommand(SuperstructureGoal.SCORE_L1.elevatorHeight));
  }

  public Command getAutonomousCommand() {
    return Path.testMoveForward(this).cmd();
  }
}
