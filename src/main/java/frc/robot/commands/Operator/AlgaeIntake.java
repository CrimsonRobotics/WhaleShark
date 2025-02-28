// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Intake.Intaking;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntake extends ParallelCommandGroup {
  /** Creates a new AlgaeIntake. */
  Elevator elevator;
  Intake intake;
  double position;
  public AlgaeIntake(Elevator elevator, Intake intake, double position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.intake = intake;
    this.position = position;

    addCommands(
      Commands.sequence(
        new WaitCommand(1),
        new Intaking(this.intake)
      ),
      new HoldPosition(this.elevator, this.position)
    );
  }
}
