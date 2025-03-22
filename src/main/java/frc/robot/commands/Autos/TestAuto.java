// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DriveAutoDouble;
import frc.robot.commands.Drivetrain.DriveDouble;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Elevator.PRunToPositionAuto;
import frc.robot.commands.Intake.Extend;
import frc.robot.commands.Intake.Shoot;
import frc.robot.commands.Operator.AlgaeIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  Elevator elevator;
  Intake intake;
  Drivetrain drivetrain;
  /** Creates a new TestAuto. */
  public TestAuto(Elevator elevator, Intake intake, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.elevator = elevator;
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.race(
        new WaitCommand(2.7),
        //new AlgaeIntake(this.elevator, this.intake, Constants.elevator.low_reef - 0.05)
        new HoldPosition(this.elevator, Constants.elevator.low_reef - 0.05)
      ),
      Commands.race(
        new WaitCommand(2.5),
        new DriveAutoDouble(this.drivetrain, -0.3, 0.27, 0, 1),
        new HoldPosition(this.elevator, Constants.elevator.high_reef)
      ),
      new Extend(this.intake),
      Commands.race(
        new WaitCommand(1),
        new PRunToPositionAuto(elevator, Constants.elevator.barge - 0.04)
      ),
      Commands.race(
        new WaitCommand(1.3),
        new DriveDouble(this.drivetrain, 0, 0.23, 0, 1)
      ),
      Commands.race(
        new WaitCommand(0.3),
        new Shoot(this.intake, false)
      ),
      Commands.race(
        new WaitCommand(1.3),
        new DriveDouble(this.drivetrain, 0, -0.26, 0, 1)
      ),
      Commands.race(
        new WaitCommand(1),
        new HoldPosition(this.elevator, Constants.elevator.coral),
        new DriveDouble(this.drivetrain, 0, -0.20, 0, 1)
      )
    );
  }
}
