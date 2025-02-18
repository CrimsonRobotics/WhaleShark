// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Travel extends Command {
  /** Creates a new Travel. */
  Intake intake;
  public Travel(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pushes piston out
    this.intake.extend();
    //configures spark flex to travel(this is the same as holding the algae)
    this.intake.configure(Constants.intake.state.HOLD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //slowly spins the motor to hold the ball in
    this.intake.run(Constants.intake.hold_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
