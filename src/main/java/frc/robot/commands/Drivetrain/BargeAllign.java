// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeAllign extends Command {
  /** Creates a new BargeAllign. */
  Vision vision;
  Drivetrain dt;
  Joystick operator_l;
  double x;
  double y;
  double rotation;
  double yaw;
  SlewRateLimiter x_limiter = new SlewRateLimiter(3);
  SlewRateLimiter y_limiter = new SlewRateLimiter(3);
  Translation2d translation;
  PIDController pid = new PIDController(Constants.Vision.barge.kp, Constants.Vision.barge.ki, Constants.Vision.barge.kd);
  PIDController rot_pid = new PIDController(Constants.dt.rot_kd, Constants.dt.rot_ki, Constants.dt.rot_kp);
  public BargeAllign(Drivetrain dt, Vision vision, Joystick operator_l) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.dt = dt;
    addRequirements(this.dt, this.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //slew rate limiter slows how fast the values change; it takes 1/3 of a second to go from 0 to 1
    //the deadband makes it so if joystick is between -0.1 and 0.1 it will just return 0
    //camera is on the side of the robot
    //the z value for the april tag is pointing towards the robot, and the x value of the april tag is pointing to the right of the robot

    y = y_limiter.calculate(MathUtil.applyDeadband(-this.operator_l.getY(), 0.1));
    x = x_limiter.calculate(pid.calculate(vision.get_target_pose()[2], Units.feetToMeters(3)));


    //using yaw to set rotation value
    // the maximum speed of rotaiton is going to be half fo the max angular velocity
    //also this will happen up utnil 90 degree away rotation, then it will start slowing down
    yaw = dt.get_yaw().getDegrees();
    rotation = MathUtil.clamp(rot_pid.calculate(yaw, 180), -90, 90) * Constants.dt.max_angular_speed / 2;


    //creates the translation value for the robot
    translation = new Translation2d(x, y).times(Constants.dt.max_speed/3);
    //drives the robot field centric
    this.dt.drive(translation, rotation, true);

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
