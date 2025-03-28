// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  Intake intake;
  boolean processor_true;
  public Shoot(Intake intake, boolean processor_true) {
    this.intake = intake;
    this.processor_true = processor_true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.extend();
    //configures spark flex to shoot algae(same configuration as intaking algae)
    this.intake.configure(Constants.intake.state.INTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //quickly spins motor to shoot algae
    if (this.processor_true) {
      this.intake.run(Constants.intake.processor_speed);
    } else {
      this.intake.run(Constants.intake.shoot_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
