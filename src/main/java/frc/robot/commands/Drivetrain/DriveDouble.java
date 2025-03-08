// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveDouble extends Command {
  /** Creates a new Drive. */
  Drivetrain dt;
  double x;
  double y;
  double rot;
  Translation2d translation;
  double rotation;
  double dt_x;
  double dt_y;
  double max_speed;
  SlewRateLimiter x_limiter = new SlewRateLimiter(3);
  SlewRateLimiter y_limiter = new SlewRateLimiter(3);
  SlewRateLimiter rotation_limiter = new SlewRateLimiter(3);

  public DriveDouble(Drivetrain dt_imp, double x, double y, double rot, double max_speed) {
    this.dt = dt_imp;
    this.x = x;
    this.y = y;
    this.rot = rot;
    this.max_speed = max_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //slew rate limiter slows how fast the values change; it takes 1/3 of a second to go from 0 to 1
    //the deadband makes it so if joystick is between -0.1 and 0.1 it will just return 0
    dt_x = x_limiter.calculate(MathUtil.applyDeadband(this.y, 0.1));
    dt_y = -y_limiter.calculate(MathUtil.applyDeadband(this.x, 0.1));
    //turns the rotation from a magnitude of 0 to 1 to be in correct speed range using the multiplication
    rotation = rotation_limiter.calculate(MathUtil.applyDeadband(this.rot, 0.1)) * Constants.dt.max_angular_speed;
    //gets robot translation and rotation from joysticks
    //.times multiplies the translation which has a magnitude between 0 and 1 inclusive by the max speed of the robot
    translation = new Translation2d(dt_x * this.max_speed, dt_y * this.max_speed).times(Constants.dt.max_speed); 

    //sets the module speeds and positions using the joystick values
    this.dt.drive(translation, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turns the rotation from a magnitude of 0 to 1 to be in correct speed range using the multiplication
    rotation = rotation_limiter.calculate(MathUtil.applyDeadband(this.rot, 0.1)) * Constants.dt.max_angular_speed;
    //gets robot translation and rotation from joysticks
    //.times multiplies the translation which has a magnitude between 0 and 1 inclusive by the max speed of the robot
    translation = new Translation2d(0, 0).times(Constants.dt.max_speed); 

    //sets the module speeds and positions using the joystick values
    this.dt.drive(translation, rotation, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
