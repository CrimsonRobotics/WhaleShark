// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Operator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.Extend;
import frc.robot.commands.Intake.Rest;
import frc.robot.commands.Intake.RunRoller;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Resting extends SequentialCommandGroup {
  /** Creates a new Rest. */
  Intake intake;
  double time;
  public Resting(Intake intake) {
    //sets the paramaters to local instance variables
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (this.intake.ground_true == true) {
      time = 2;
    } else {
      time = 1;
    }
    SmartDashboard.putNumber("Time", time);

    //these commands will both happen at the same time
    addCommands(
      //pulls the intake up
      new Extend(this.intake),
      //spins the intake rollers for a second while intake goes up
      Commands.race(
        new WaitCommand(this.intake.time_update()),
        new RunRoller(this.intake, Constants.intake.intake_speed)
      ),
      //has the intake stay in rest mode
      new Rest(this.intake)
    );
  }
}
