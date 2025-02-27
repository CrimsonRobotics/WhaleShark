// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class dt {
    public static class mod0 {
      public static final int drive_id = 39;
      public static final int turn_id = 43;
      public static final int can_coder = 5;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(67.3);
    }
    public static class mod1 {
      public static final int drive_id = 60;
      public static final int turn_id = 24;
      public static final int can_coder = 6;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(271.2);
    }
    public static class mod2 {
      public static final int drive_id = 48;
      public static final int turn_id = 38;
      public static final int can_coder = 4;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(103.8);
    }
    public static class mod3 {
      public static final int drive_id = 45;
      public static final int turn_id = 32;
      public static final int can_coder = 7;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(226.0);
    }

    //this is the drive motor gear ratio, basically 6 rotation of the motor will rotation the drivec motor once
    public static final double drive_motor_ratio = 6.75;
    //this is how many ecoder ticks for one motor revolution
    public static final double encoder_tick_ratio = 1;
    //this is the diameter of the drive wheel in meters, the number put in is in inches
    public static final double wheel_diameter = Units.inchesToMeters(4);
    //this is the gear ratio of the turn motor, it takes 150 revolutions of the turn motor to rotate the wheel 7 times
    public static final double turn_motor_ratio = 150.0 / 7.0;
    //this uses the constants of the module to create a drive position vonversion factor.
    //The goal is to have the encoder read in meters not ticks
    public static final double drive_position_conversion_factor = wheel_diameter * Math.PI / (encoder_tick_ratio * drive_motor_ratio); //Math.PI * drive_motor_ratio * 9.5
    public static final double drive_velocity_conversion_factor = drive_position_conversion_factor / 60.0;
    public static final double turn_position_conversion_factor = 360.0 / turn_motor_ratio;

    //lengths in nches to meters
    public static final double robot_length = Units.inchesToMeters(21.5);
    public static final double robot_width = Units.inchesToMeters(21.5);

    public static final double turn_kp = 0.00759; //0.00759;
    public static final double turn_ki = 0.00069; //0.00069;
    public static final double turn_kd = 0.0001;  //0.0001;
    public static final double drive_kp = 0.02;
    public static final double drive_ki = 0;
    public static final double drive_kd = 0;

    //max values in meters per second or radians per second
    public static final double max_speed = 5;
    public static final double max_angular_speed = 7.0;

    //front of robot is postive x and back of robot is negative x
    //left of robot is positive y and right of robot is negative y
    public static final Translation2d front_right = new Translation2d(-robot_length / 2, -robot_width / 2);
    public static final Translation2d back_right = new Translation2d(-robot_length / 2, robot_width / 2);
    public static final Translation2d back_left = new Translation2d(robot_length / 2, robot_width / 2);
    public static final Translation2d front_left = new Translation2d(robot_length / 2, -robot_width / 2);
    //swerve drice kinematics
    public static final SwerveDriveKinematics swerve_map = new SwerveDriveKinematics(
      front_right,
      back_right,
      back_left,
      front_left
    );

    public static final double rot_kp = .01111;
    public static final double rot_ki = .000001;
    public static final double rot_kd = 0;
  }

  public static class elevator {
    public static final int r_motor_id = 41;
    public static final int l_motor_id = 40;
    //run to position pid values
    public static final double kp = 0.4;
    public static final double ki = .04;
    public static final double kd = 0.01;
    //height values of the elvator
    public static final double low_reef = 0.71;
    public static final double high_reef = 42.375;
    public static final double coral = 11.375;
    public static final double ground = .019;
    public static final double rest = .019;
    public static final double barge = 2.207;
    public static final double travel = .019;

    //conversion factors for the elevator
    //stage position change to chain movement
    public static final double stage_to_chain = 3;
    //gear ratio in first gear box
    public static final double gear_box_ratio = 5;
    //gear ratio after encoder
    //58 rotations of encoder shaft equals 16 rotations of output shaft
    public static final double  gear_ratio = 58 / 16;
    //encoder ticks per rotation of shaft
    public static final double encoder_ticks = 1;
    //diameter of sprocket with chain in meters
    public static final double diameter = Units.inchesToMeters(1.751);
    //rotations of output shaft to meters of elevator movement
    public static final double rotations_to_meters = diameter * Math.PI;
    public static final double position_conversion_factor = Units.inchesToMeters(35.625) / 39.1663;
    public static final double velocity_conversion_factor = position_conversion_factor / 60;
    

    //feed forward input amount
    public static final double feed_forward_amount = 0.035;

    //elevator speeds
    public static final double high_speed = 1;
    public static final double slow_speed = 0.5;
  }

  public static class climber {
    public static final int motor_id = 35;

    //run to position pid values
    public static final double kp = 0.1;
    public static final double ki = 0;
    public static final double kd = 0;

    //position coversion factor stuff for climber motor
    //gear ratio of gear box
    //343 rotations of motor equals 1 rotation of the climber output shaft
    public static final double gear_ratio = 343;
    public static final double encoder_ticks = 1;
    public static final double position_conversion_factor = 1 / (gear_ratio * encoder_ticks);

    //ready up position is the position that the climber will be at when it is ready to climb
    //this is in rotations of the motor
    public static final double ready_up_position = 0.25;
    //this is the speed at which the robot will climb
    public static final double climb_voltage = 1.5;


  }

  public static class intake {
    public static final int motor_id = 10;
    public static final int hold_current_limit = 10;
    public static final int intake_current_limit = 60;

    //intake motor spin speeds
    public static final double hold_speed = 0.1;
    public static final double intake_speed = 0.8;
    public static final double shoot_speed = -0.5;
    public static final double rest_speed = 0.1;

    //enum for configs
    //an enum is a variable that you create with set values you define
    //it is helpful because it is like a boolean but can have more than two values
    //it is better than number because it does not take up as much storage and ram
    //and it is better than both because it is more descriptive
    //I am using it for switching states of the intake
    public static enum state {
      HOLD,
      INTAKE
    }
  }
  
}
