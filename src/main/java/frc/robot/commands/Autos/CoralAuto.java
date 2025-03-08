// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Intake.CoralDrop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralAuto extends ParallelRaceGroup {
  /** Creates a new CoralAuto. */
  Drivetrain drivetrain;
  Intake intake;
  Elevator elevator;
  public CoralAuto(Drivetrain drivetrain, Intake intake, Elevator elevator) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.elevator = elevator;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new HoldPosition(this.elevator, Constants.elevator.coral_auto),
      Commands.sequence(
        new DriveTime(this.drivetrain, Constants.autos.coral_drive_time),
        Commands.race(
          new WaitCommand(Constants.autos.coral_score_time),
          new CoralDrop(this.intake)
        ),
        new WaitCommand(3)
      )
    );
  }
}
