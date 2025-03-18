// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5274.lib.util.ConditionalUitls;
import team5274.lib.util.TactileAlert;
import team5274.robot.subsystems.ClimberClamp;
import team5274.robot.subsystems.Superstructure;
import team5274.robot.subsystems.Superstructure.SuperstructurePose;
import team5274.robot.subsystems.arm.Arm;
import team5274.robot.subsystems.arm.Pincer;
import team5274.robot.subsystems.drive.Drive;
import team5274.robot.subsystems.elevator.Elevator;
import team5274.robot.subsystems.elevator.ElevatorPivot;

public class RobotContainer {

  public final static SendableChooser<Boolean> modeChooser = new SendableChooser<>();
  public static SuperstructurePose _robotPose = SuperstructurePose.IDLE;
  public final SendableChooser<Command> autoChooser;

  public final static CommandXboxController driverController = new CommandXboxController(0);
  public final static CommandXboxController operatorController = new CommandXboxController(1);

  public ElevatorPivot elevatorPivot = ElevatorPivot.get();
  public Elevator elevator = Elevator.get();
  public static Trigger elevatorAtLowest;

  public Arm arm = Arm.get();
  public Pincer pincer = Pincer.get();

  public Drive drive = Drive.get();

  public RobotContainer() {
    modeChooser.setDefaultOption("Debug Mode", true);
    modeChooser.setDefaultOption("Comp Mode", false);
    modeChooser.setDefaultOption("Comp Mode", false);
    modeChooser.onChange(this::initSystems);
    SmartDashboard.putData("Mode", modeChooser);
    
    NamedCommands.registerCommand("PoseTrough", Superstructure.pose(this, SuperstructurePose.SCORE_TROUGH));
    NamedCommands.registerCommand("ShortDeposit", pincer.dutyCycleCommand(() -> -0.25).withTimeout(1));

    configureBindings();

    // new Trigger(() -> operatorController.getRightTriggerAxis() >= 0.5 || operatorController.getLeftTriggerAxis() >= 0.5).whileTrue(new TactileAlert(operatorController));
    //Initial Endgame warning (2 pulses at 20 seconds match time)
    new Trigger(
      () -> DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 20
    )
    .onTrue(
      new TactileAlert(driverController)
        .withTimeout(0.5)
        .andThen(Commands.waitSeconds(0.1))
        .repeatedly()
        .withTimeout(1.1)
        .alongWith(
          new TactileAlert(operatorController)
            .withTimeout(0.5)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(1.1))
    );

    //Second warning (5 pulses at 10 seconds match time)
    new Trigger(
      () -> DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 10
    )
    .onTrue(
      new TactileAlert(driverController)
        .withTimeout(0.1)
        .andThen(Commands.waitSeconds(0.1))
        .repeatedly()
        .withTimeout(1.1)
        .alongWith(
          new TactileAlert(operatorController)
            .withTimeout(0.1)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(1.1))
    );

    //Final Warning (3 long pulses)
    new Trigger(
      () -> DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 3
    )
    .onTrue(
      new TactileAlert(driverController)
        .withTimeout(0.9)
        .andThen(Commands.waitSeconds(0.1))
        .repeatedly()
        .withTimeout(2.9)
        .alongWith(
          new TactileAlert(operatorController)
            .withTimeout(0.9)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(2.9))
    );

    elevatorAtLowest = new Trigger(elevator::isAtLowestValid);
    elevatorAtLowest.onTrue(elevator.homeCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);
    SmartDashboard.putData(modeChooser);
  }

  private void configureBindings() {
    driverController.start().onTrue(drive.reset());
   driverController.rightBumper().and(() -> _robotPose == SuperstructurePose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperstructurePose.ALGAE_L2));
    driverController.leftBumper().and(() -> _robotPose == SuperstructurePose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperstructurePose.ALGAE_L3));

    driverController.a().onTrue(drive.getDefaultCommand());

    operatorController.a().onTrue(Superstructure.pose(this, SuperstructurePose.IDLE));
    operatorController.y().and(() -> _robotPose == SuperstructurePose.IDLE).toggleOnTrue(Superstructure.pose(this, SuperstructurePose.INTAKE_STATION));

    operatorController.x().and(() -> _robotPose == SuperstructurePose.PREP_L1).onTrue(Superstructure.pose(this, SuperstructurePose.SCORE_L1));
    operatorController.x().and(() -> _robotPose == SuperstructurePose.PREP_L2).onTrue(Superstructure.pose(this, SuperstructurePose.SCORE_L2));
    operatorController.x().and(() -> _robotPose == SuperstructurePose.PREP_L3).onTrue(Superstructure.pose(this, SuperstructurePose.SCORE_L3));

    operatorController.pov(0).and(() -> _robotPose == SuperstructurePose.IDLE).onTrue(Superstructure.pose(this, SuperstructurePose.PREP_L3));
    operatorController.pov(90).and(() -> _robotPose == SuperstructurePose.IDLE).onTrue(Superstructure.pose(this, SuperstructurePose.PREP_L2));
    operatorController.pov(180).and(() -> _robotPose == SuperstructurePose.IDLE).onTrue(Superstructure.pose(this, SuperstructurePose.SCORE_TROUGH));
    operatorController.pov(270).and(() -> _robotPose == SuperstructurePose.IDLE).onTrue(Superstructure.pose(this, SuperstructurePose.PREP_L1));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public boolean getIntakeInputInflection() {
    return ConditionalUitls.withinTolerance(operatorController.getRightTriggerAxis(), 0.5, 0.1);
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
}
