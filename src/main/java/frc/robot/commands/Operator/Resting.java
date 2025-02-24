// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Intake.Rest;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Resting extends ParallelCommandGroup {
  /** Creates a new Rest. */
  Elevator elevator;
  Intake intake;
  public Resting(Elevator elevator, Intake intake) {
    //sets the paramaters to local instance variables
    this.elevator = elevator;
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //these commands will both happen at the same time
    addCommands(
      //the elevator will hold positiot at rest height
      Commands.sequence(
        new WaitCommand(1),
        new HoldPosition(this.elevator, Constants.elevator.rest)
      ),
      //the intake will go into rest state
      new Rest(this.intake)
    );
  }
}
