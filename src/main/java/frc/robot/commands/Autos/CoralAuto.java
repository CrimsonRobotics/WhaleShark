// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.CoralDrop;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralAuto extends SequentialCommandGroup {
  /** Creates a new CoralAuto. */
  Drivetrain drivetrain;
  Intake intake;
  public CoralAuto(Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveTime(this.drivetrain, 2),
      Commands.race(
        new WaitCommand(1),
        new CoralDrop(this.intake)
      )
    );
  }
}
