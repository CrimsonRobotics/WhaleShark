// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Elevator.RunToPosition;
import frc.robot.commands.Intake.Intaking;
import frc.robot.commands.Intake.UpdateDefault;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntake extends SequentialCommandGroup {
  /** Creates a new AlgaeIntake. */
  Elevator elevator;
  Intake intake;
  double position;
  boolean ground_intake;
  public AlgaeIntake(Elevator elevator, Intake intake, double position, boolean ground_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.elevator = elevator;
    this.intake = intake;
    this.position = position;
    this.ground_intake = ground_intake;
    

    addCommands(
      new UpdateDefault(this.intake, this.ground_intake),
      new RunToPosition(this.elevator, this.position),
      Commands.parallel(
        new HoldPosition(this.elevator, this.position),
        new Intaking(this.intake)
      )
    );
  }
}
