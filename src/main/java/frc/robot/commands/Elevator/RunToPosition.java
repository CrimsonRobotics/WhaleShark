// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunToPosition extends Command {
  /** Creates a new HoldPosition. */
  Elevator elevator;
  double position;
  public RunToPosition(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.run_to_position(this.position);
    SmartDashboard.putNumber("Current Elevator Position", this.elevator.get_position());
    SmartDashboard.putNumber("setpoint elevator", this.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.elevator.get_position() > this.position - 0.02 && this.elevator.get_position() < this.position + 0.02) {
      SmartDashboard.putBoolean("Elevator Loop", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Elevator Loop", false);
      return false;
    }

  }
}
