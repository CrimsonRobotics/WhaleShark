// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.Drive;
import frc.robot.commands.Drivetrain.DriveDouble;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveTime extends ParallelRaceGroup {
  /** Creates a new DriveTime. */
  double seconds;
  Drivetrain drivetrain;
  public DriveTime(Drivetrain dt, double seconds) {
    this.seconds = seconds;
    this.drivetrain = dt;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(this.seconds),
      new DriveDouble(this.drivetrain, 0, 0.25, 0, 1)
    );
  }
}
