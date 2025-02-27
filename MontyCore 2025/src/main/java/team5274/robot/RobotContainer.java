// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team5274.robot.subsystems.Superstructure;
import team5274.robot.subsystems.Superstructure.SuperstructureGoal;
import team5274.robot.subsystems.arm.Arm;
import team5274.robot.subsystems.arm.Pincer;
import team5274.robot.subsystems.drive.Drive;
import team5274.robot.subsystems.elevator.Elevator;
import team5274.robot.subsystems.elevator.ElevatorPivot;

public class RobotContainer {

  public final static boolean debugMode = false;
  public static SuperstructureGoal currentGoal = SuperstructureGoal.IDLE;
  public final SendableChooser<Command> autoChooser;

  public final static CommandXboxController driverController = new CommandXboxController(0);
  public final static CommandXboxController operatorController = new CommandXboxController(1);

  public ElevatorPivot elevatorPivot = ElevatorPivot.get();
  public Elevator elevator = Elevator.get();

  public Arm arm = Arm.get();
  public Pincer pincer = Pincer.get();

  public Drive drive = Drive.get();

  public RobotContainer() {
    NamedCommands.registerCommand("PoseTrough", Superstructure.pose(this, () -> SuperstructureGoal.SCORE_TROUGH));
    NamedCommands.registerCommand("ShortDeposit", pincer.dutyCycleCommand(() -> -0.3).withTimeout(2));

    configureBindings();

    if(debugMode) {
      elevatorPivot.setDefaultCommand(elevatorPivot.dutyCycleCommand(() -> -operatorController.getLeftY()));
      elevator.setDefaultCommand(elevator.dutyCycleCommand(() -> -operatorController.getRightY()));
      arm.setDefaultCommand(arm.dutyCycleCommand(
        () -> operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis(),
        () -> operatorController.pov(90).getAsBoolean() ? 0.05 : operatorController.pov(270).getAsBoolean() ? -0.05 : 0
      ));
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {

    driverController.start().onTrue(drive.reset());
    operatorController.start().toggleOnTrue(Superstructure.pose(this, () -> SuperstructureGoal.DEBUG));

    operatorController.a().toggleOnTrue(Superstructure.pose(this, () -> SuperstructureGoal.IDLE));
    operatorController.y().toggleOnTrue(Superstructure.pose(this, () -> SuperstructureGoal.INTAKE_STATION));

    operatorController.x().and(() -> currentGoal == SuperstructureGoal.PREP_L1).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.SCORE_L1)));
    operatorController.x().and(() -> currentGoal == SuperstructureGoal.PREP_L2).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.SCORE_L2)));
    operatorController.x().and(() -> currentGoal == SuperstructureGoal.PREP_L3).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.SCORE_L3)));


    operatorController.pov(0).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.PREP_L3)));
    operatorController.pov(90).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.PREP_L2)));
    operatorController.pov(180).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.SCORE_TROUGH)));
    operatorController.pov(270).onTrue(Superstructure.pose(this, () -> Superstructure.interceptAndModifyGoal(SuperstructureGoal.PREP_L1)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
