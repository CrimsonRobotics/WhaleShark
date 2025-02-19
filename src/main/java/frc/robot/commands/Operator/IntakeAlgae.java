// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Elevator.HoldPosition;
import frc.robot.commands.Elevator.RunToPosition;
import frc.robot.commands.Intake.Intaking;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAlgae extends SequentialCommandGroup {
  /** Creates a new LowReef. */
  Elevator elevator;
  Intake intake;
  double position;
  public IntakeAlgae(Elevator elevator, Intake intake, double position) {
    //sets the paramaters to local instance variables
    this.elevator = elevator;
    this.intake = intake;
    this.position = position;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //this command will end when another command interrupts it or when the button is let go because the last commands have no built in end
    
    //does these commands in order
    addCommands(
      //first the elevator will run to its designated position and the command will end
      new RunToPosition(this.elevator, this.position),
      //next these two commands will happen at the same time
      Commands.parallel(
        //the intake will go into its intake state
        new Intaking(this.intake),
        //the elevator will hold its position at the designated position
        new HoldPosition(this.elevator, this.position)
      )
    );
  }
}
