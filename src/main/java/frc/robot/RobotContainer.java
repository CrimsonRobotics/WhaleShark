// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Operator.BargeScore;
import frc.robot.commands.Operator.IntakeAlgae;
import frc.robot.commands.Operator.Resting;
import frc.robot.commands.Operator.Traveling;
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

  //Left Operator Joystick
  //Button 8
  private final JoystickButton resting = new JoystickButton(l_operator, 8);
  //Button 9
  private final JoystickButton traveling = new JoystickButton(l_operator, 9);

  //Right Operator Joystick
  //Button 12
  private final JoystickButton coral_intake = new JoystickButton(r_operator, 12);
  //Button 13
  private final JoystickButton ground_intake = new JoystickButton(r_operator, 13);
  //Button 14
  private final JoystickButton low_reef_intake = new JoystickButton(r_operator, 14);
  //Button 15
  private final JoystickButton high_reef_intake = new JoystickButton(r_operator, 15);
  //Button 16
  private final JoystickButton barge_score = new JoystickButton(r_operator, 16);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
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
    //left operator buttons
    resting.onTrue(new InstantCommand(() -> elevator.setDefaultCommand(new Resting(elevator, intake))));
    traveling.onTrue(new InstantCommand(() -> elevator.setDefaultCommand(new Traveling(elevator, intake))));
    //right operator buttons
    coral_intake.whileTrue(new IntakeAlgae(elevator, intake, Constants.elevator.coral));
    ground_intake.whileTrue(new IntakeAlgae(elevator, intake, Constants.elevator.ground));
    low_reef_intake.whileTrue(new IntakeAlgae(elevator, intake, Constants.elevator.low_reef));
    high_reef_intake.whileTrue(new IntakeAlgae(elevator, intake, Constants.elevator.high_reef));
    barge_score.whileTrue(new BargeScore(elevator, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
