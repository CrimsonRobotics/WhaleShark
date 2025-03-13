// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoFile;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Autos.CoralAuto;
import frc.robot.commands.Autos.DriveTime;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimbBack;
import frc.robot.commands.Climber.ReadyUp;
import frc.robot.commands.Drivetrain.Drive;
import frc.robot.commands.Elevator.ElevatorNothing;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Elevator.PRunToPosition;
import frc.robot.commands.Elevator.RunElevator;
import frc.robot.commands.Intake.Extend;
import frc.robot.commands.Intake.Intaking;
import frc.robot.commands.Intake.Rest;
import frc.robot.commands.Intake.Retract;
import frc.robot.commands.Intake.RunRoller;
import frc.robot.commands.Intake.Shoot;
import frc.robot.commands.Operator.AlgaeIntake;
import frc.robot.commands.Operator.Resting;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final Drivetrain drivetrain = new Drivetrain();

  //Creating joysticks
  private final Joystick l_drive = new Joystick(0);
  private final Joystick r_drive = new Joystick(1);
  private final Joystick l_operator = new Joystick(2);
  private final Joystick r_operator = new Joystick(3);
  
  //Creating Joystick Buttons
  //Left Driver Joystick
  //Button 1
  private final JoystickButton reset_gyro = new JoystickButton(l_drive, 1);
  //Button 2
  private final JoystickButton slow_drive = new JoystickButton(l_drive, 2);

  //Right Driver Joystick

  //Left Operator Joystick
  //Button 1
  private final JoystickButton ground_intake = new JoystickButton(l_operator, 1);
  //Button 2
  private final JoystickButton low_reef_intake = new JoystickButton(l_operator, 2);
  //Button 3
  private final JoystickButton high_reef_intake = new JoystickButton(l_operator, 3);
  //Button 4
  private final JoystickButton coral_intake = new JoystickButton(l_operator, 4);
  //Button 5
  private final JoystickButton reset_elevator = new JoystickButton(l_operator, 5);

  //Right Operator Joystick
  //Button 1
  private final JoystickButton shoot_intake = new JoystickButton(r_operator, 1);
  //Button 2
  private final JoystickButton barge_elevator_control = new JoystickButton(r_operator, 2);
  //Button 3
  private final JoystickButton run_elevator = new JoystickButton(r_operator, 3);
  //Buttom 8
  private final JoystickButton intake_only = new JoystickButton(r_operator, 8);
  



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    elevator.setDefaultCommand(new RunElevator(elevator, r_operator, 1));
    drivetrain.setDefaultCommand(new Drive(drivetrain, l_drive, r_drive, Constants.driver.normal_speed));
    intake.setDefaultCommand(new Rest(intake));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());


    //configuring buttons
    //left driver buttons
    reset_gyro.onTrue(new InstantCommand(() -> drivetrain.set_gyro(-90)));
    slow_drive.whileTrue(new Drive(drivetrain, l_drive, r_drive, Constants.driver.slow_speed));

    //right driver buttons
    //run_roller.whileTrue(new RunRoller(intake, r_drive));
    //piston_extend.onTrue(new Extend(intake));
    //piston_retract.onTrue(new Retract(intake));

    //left operator buttons
    ground_intake.whileTrue(new AlgaeIntake(elevator, intake, Constants.elevator.ground));
    low_reef_intake.whileTrue(new AlgaeIntake(elevator, intake, Constants.elevator.low_reef));
    high_reef_intake.whileTrue(new AlgaeIntake(elevator, intake, Constants.elevator.high_reef));
    coral_intake.whileTrue(new AlgaeIntake(elevator, intake, Constants.elevator.coral));
    reset_elevator.onTrue(new InstantCommand(() -> elevator.reset_elevator()));


    ////right operator buttons
    shoot_intake.whileTrue(new Shoot(intake));
    barge_elevator_control.whileTrue(new PRunToPosition(elevator, Constants.elevator.barge, r_operator));
    run_elevator.whileTrue(new RunElevator(elevator, r_operator, Constants.elevator.high_speed));
    intake_only.whileTrue(new Intaking(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new CoralAuto(drivetrain, intake, elevator);
  }
}
