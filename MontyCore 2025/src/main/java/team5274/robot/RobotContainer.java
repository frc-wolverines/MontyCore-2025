// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5274.lib.util.TactileAlert;
import team5274.robot.subsystems.Superstructure;
import team5274.robot.subsystems.Superstructure.SuperPose;
import team5274.robot.subsystems.arm.Arm;
import team5274.robot.subsystems.arm.Affector;
import team5274.robot.subsystems.drive.Drive;
import team5274.robot.subsystems.elevator.Elevator;
import team5274.robot.subsystems.elevator.ElevatorPivot;

public class RobotContainer {
  public static SuperPose pose = SuperPose.IDLE;

  public static SendableChooser<Boolean> modeChooser;
  public static SendableChooser<Command> autoChooser;  

  public final static CommandXboxController driverController = new CommandXboxController(0);
  public final static CommandXboxController operatorController = new CommandXboxController(1);

  public ElevatorPivot elevatorPivot = ElevatorPivot.get();
  public Elevator elevator = Elevator.get();
  public Trigger elevatorAtLowest;
  
  public Arm arm = Arm.get();
  public Affector pincer = Affector.get();
  
  public Drive drive = Drive.get();

  public RobotContainer() {
    configureSettings();
    configureBindings();
    configureAuto();
    configureEvents();
  }

  private void configureEvents() {
    setupAlert(0, 20, 0.5, 2); //Initial Endgame warning (2 pulses at 20 seconds match time)
    setupAlert(0, 10, 0.1, 5); //Second warning (5 pulses at 10 seconds match time)
    setupAlert(0, 3, 0.9, 3); //Final Warning (3 long pulses)

    //Setup the elevator homing trigger
    elevatorAtLowest = new Trigger(elevator::isAtLowestValid);
    elevatorAtLowest.onTrue(elevator.homeCommand());
  }

  private void configureSettings() {
    modeChooser = new SendableChooser<>();
    modeChooser.setDefaultOption("Debug Mode", true);
    modeChooser.setDefaultOption("Comp Mode", false);
    modeChooser.setDefaultOption("Comp Mode", false);
    modeChooser.onChange(this::initSystems);
    SmartDashboard.putData("Mode", modeChooser);
  }

  private void configureAuto() {
    NamedCommands.registerCommand("PoseTrough", Superstructure.pose(this, SuperPose.SCORE_TROUGH));
    NamedCommands.registerCommand("ShortDeposit", pincer.dutyCycleCommand(() -> -0.25).withTimeout(1));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
  }

  private void configureBindings() {
    driverController.start().onTrue(drive.reset());
    driverController.rightBumper().and(() -> pose == SuperPose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperPose.ALGAE_L2));
    driverController.leftBumper().and(() -> pose == SuperPose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperPose.ALGAE_L3));

    driverController.a().onTrue(drive.getDefaultCommand());

    operatorController.a().onTrue(Superstructure.pose(this, SuperPose.IDLE));
    operatorController.y().and(() -> pose == SuperPose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperPose.INTAKE_STATION));

    operatorController.x().and(() -> pose == SuperPose.PREP_L1).onTrue(Superstructure.pose(this, SuperPose.SCORE_L1));
    operatorController.x().and(() -> pose == SuperPose.PREP_L2).onTrue(Superstructure.pose(this, SuperPose.SCORE_L2));
    operatorController.x().and(() -> pose == SuperPose.PREP_L3).onTrue(Superstructure.pose(this, SuperPose.SCORE_L3));

    operatorController.pov(0).and(() -> pose == SuperPose.IDLE).onTrue(Superstructure.pose(this, SuperPose.PREP_L3));
    operatorController.pov(90).and(() -> pose == SuperPose.IDLE).onTrue(Superstructure.pose(this, SuperPose.PREP_L2));
    operatorController.pov(180).and(() -> pose == SuperPose.IDLE).onTrue(Superstructure.pose(this, SuperPose.SCORE_TROUGH));
    operatorController.pov(270).and(() -> pose == SuperPose.IDLE).onTrue(Superstructure.pose(this, SuperPose.PREP_L1));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void initSystems(boolean debugMode) {
    if(debugMode) {
      elevatorPivot.setDefaultCommand(elevatorPivot.dutyCycleCommand(() -> -operatorController.getLeftY()));
      elevator.setDefaultCommand(elevator.dutyCycleCommand(() -> -operatorController.getRightY()));
      arm.setDefaultCommand(arm.dutyCycleCommand(
        () -> operatorController.getLeftTriggerAxis() * 0.15 - operatorController.getRightTriggerAxis() * 0.15,
        () -> 0.0
      ));
    } else {
      elevatorPivot.setDefaultCommand(elevatorPivot.persistantAngleCommand(() -> elevatorPivot.cachedAngle));
      elevator.setDefaultCommand(elevator.persistantHeightCommand(() -> elevator.cachedHeight));
      arm.setDefaultCommand(arm.persistantAngleCommand(() -> arm.cachedArmAngle, () -> arm.cachedWristAngle));
    }
  }

  public void setupAlert(double minTime, double maxTime, double pulseWidth, double pulseCount) {
    double alertTimeout = (pulseWidth * pulseCount) + (0.1 * (pulseCount - 1));

    new Trigger(
      () -> DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > minTime
        && DriverStation.getMatchTime() <= maxTime
    )
    .onTrue(
      new TactileAlert(driverController)
        .withTimeout(pulseWidth)
        .andThen(Commands.waitSeconds(0.1))
        .repeatedly()
        .withTimeout(alertTimeout)
        .alongWith(
          new TactileAlert(operatorController)
            .withTimeout(pulseWidth)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(alertTimeout)));
  }
}
