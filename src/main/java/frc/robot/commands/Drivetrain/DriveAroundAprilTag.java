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
import frc.robot.subsystems.Tracking;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveAroundAprilTag extends Command {
  /** Creates a new DriveAroundAprilTag. */
  Drivetrain dt;
  Tracking track;
  Joystick joystick_ol;
  Joystick joystick_dl;
  Translation2d translation;
  double rotation;
  double dt_x;
  double dt_y;
  double max_speed;
  SlewRateLimiter x_limiter = new SlewRateLimiter(3);
  SlewRateLimiter y_limiter = new SlewRateLimiter(3);
  SlewRateLimiter rotation_limiter = new SlewRateLimiter(3);
  public DriveAroundAprilTag(Drivetrain dt_imp, Tracking track, Joystick joystick_dl, Joystick joystick_ol, double max_speed) {
    this.dt = dt_imp;
    this.joystick_ol = joystick_ol;
    this.joystick_dl = joystick_dl;
    this.max_speed = max_speed;
    this.track = track;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt);
    addRequirements(track);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_speed = x_limiter.calculate(MathUtil.applyDeadband(this.joystick_dl.getY(), 0.1)) * Constants.dt.max_speed;
    double y_speed = y_limiter.calculate(MathUtil.applyDeadband(this.joystick_dl.getX(), 0.1)) * Constants.dt.max_speed;

    this.dt.drive(x_speed, y_speed, -Tracking.limelight_aim_proportional(), false);
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
