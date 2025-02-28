// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.Autos.DriveTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class AutoFile {
  /** Example static factory for an autonomous command. */
  public static Command Leave(Drivetrain drivetrain) {
    return new DriveTime(drivetrain, 2);
  }

  private AutoFile() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
