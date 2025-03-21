// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  /** Creates a new RunElevator. */
  Elevator elevator;
  Joystick joystick;
  double max_speed;
  public RunElevator(Elevator elevator, Joystick joystick, double max_speed) {
    this.elevator = elevator;
    this.joystick = joystick;
    this.max_speed = max_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Applied Input from", joystick.getX());
    SmartDashboard.putNumber("MAx Speed Elevator", this.max_speed);
    this.elevator.run(joystick.getX() * this.max_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
