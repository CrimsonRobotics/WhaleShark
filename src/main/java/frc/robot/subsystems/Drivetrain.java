// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  RobotConfig config;

  Pigeon2 gyro;
  private final SwerveModule[] dt;
  SwerveDrivePoseEstimator pose_estimator;
  Field2d field;
  public Drivetrain() {
    gyro = new Pigeon2(10);
    set_gyro(0);

    this.dt = new SwerveModule[] {
      new SwerveModule(0, Constants.dt.mod0.drive_id, Constants.dt.mod0.turn_id, Constants.dt.mod0.can_coder, Constants.dt.mod0.turn_offset),
      new SwerveModule(1, Constants.dt.mod1.drive_id, Constants.dt.mod1.turn_id, Constants.dt.mod1.can_coder, Constants.dt.mod1.turn_offset),
      new SwerveModule(2, Constants.dt.mod2.drive_id, Constants.dt.mod2.turn_id, Constants.dt.mod2.can_coder, Constants.dt.mod2.turn_offset),
      new SwerveModule(3, Constants.dt.mod3.drive_id, Constants.dt.mod3.turn_id, Constants.dt.mod3.can_coder, Constants.dt.mod3.turn_offset)
    };
    reset_encoders();

    //pose estimator to estimate where the robot is on the field using encoder and yaw data
    pose_estimator = new SwerveDrivePoseEstimator(Constants.dt.swerve_map, get_yaw(), 
      new SwerveModulePosition[] {this.dt[0].get_position(), this.dt[1].get_position(), this.dt[2].get_position(), this.dt[3].get_position()}, new Pose2d());

    //field to visualize where pose estimator thinks robot is
    field = new Field2d();

    //try to set the config from the robot config from the GUI settings in the path planner app
    try{
      config = RobotConfig.fromGUISettings();
    //if there is a error it catches it and prints it
    } catch (Exception exception){
      exception.printStackTrace();
    }
    //configures the auto builder
    AutoBuilder.configure(
      //gets a pose supplier
      this.get_pose_supplier(),
      //uses the pose supplier to reset the pose estimator
      (p) -> this.reset_supplier_pose(get_pose_supplier()),
      //gets a robot relative speeds as supplier
      this.get_robot_relative_speeds(),
      //uses the speed supplier to drive the robot 
      (speed_supplier,feedforward) -> this.drive_robot_relative_speeds(speed_supplier),
      //creates a new controller with PID Constants for driving and rotations
      new PPHolonomicDriveController(
        new PIDConstants(0, 0, 0), 
        new PIDConstants(0, 0, 0)),
      //robot configs
      config,
      //used to flip the path
      () -> {
        //creates a variable called alliance and gets the alliance color from the driver station
        var alliance = DriverStation.getAlliance();
        //origin of path is on blue side
        //if the alliance is present
        if (alliance.isPresent()){
          //flip the path to red alliance
          return alliance.get() == DriverStation.Alliance.Red;
        }
        //if not than keep the path on blue alliance
        return false;
      },
      //requirements are itself
      this);
  }

  //gets the pose as a supplier
  public Supplier<Pose2d> get_pose_supplier() {
    return () -> pose_estimator.getEstimatedPosition();
  }

  //reset the pose with the pose supplier
  public void reset_supplier_pose(Supplier<Pose2d> pose_supplier){
    Pose2d pose = pose_supplier.get();
    //takes a Pose2d and resets the pose estimator using current yaw and module position
    Consumer<Pose2d> reset = (p) -> this.pose_estimator.resetPosition(get_yaw(), 
    new SwerveModulePosition[] {
      this.dt[0].get_position(),
      this.dt[1].get_position(),
      this.dt[2].get_position(),
      this.dt[3].get_position()
    }, 
    pose);
    //accepts the pose from the pose supplier
    reset.accept(pose);
  }

  //gets a chassis speed as a supplier
  public Supplier<ChassisSpeeds> get_robot_relative_speeds(){
    //gets the supplier from the states from the modules
    return () -> Constants.dt.swerve_map.toChassisSpeeds(new SwerveModuleState[]{
      this.dt[0].get_state(),
      this.dt[1].get_state(),
      this.dt[2].get_state(),
      this.dt[3].get_state()
    });
  }
  
  //drives the robot with the chassis speeds fron get_robot_relative_speeds
  public void drive_robot_relative_speeds(ChassisSpeeds speed_supplier){
    ChassisSpeeds speeds = speed_supplier;

    //takes a chassis speed to drive
    Consumer<ChassisSpeeds> drive = (s) -> {
      //gets the module state 
      SwerveModuleState[] swerve_module_state = Constants.dt.swerve_map.toSwerveModuleStates(speeds);
      //slows the wheels to prevent it from going over max speed
      SwerveDriveKinematics.desaturateWheelSpeeds(swerve_module_state, Constants.dt.max_speed);
      //sets desired state for all modules
      for(SwerveModule module: this.dt) {
        module.set_desired_state(swerve_module_state[module.module_number]);
      }
    };
    //accepts the speed to drive the robot
    drive.accept(speeds);
  }


  public void set_gyro(double yaw) {
    //sets the gyros yaw to the yaw paramter
    //this method is in degrees
    gyro.setYaw(yaw);
  }

  //resets the encoders of each module to the cancoder offset
  public void reset_encoders() {
    for (SwerveModule module : this.dt) {
      module.reset_encoder();
    }
  }

  //returns the yaw of robot
  public Rotation2d get_yaw() {
    //gets the yaw as a Rotation 2d variable using Phoenix 6 API
    return gyro.getRotation2d();
  }

  //drive funciton
  public void drive(Translation2d translation, double rotation, boolean is_field_relative) {
    //creates swerve module states form joystick values which are in translation and rotation
    SwerveModuleState[] swerve_module_states =
      Constants.dt.swerve_map.toSwerveModuleStates(is_field_relative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), translation.getY(), rotation, get_yaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    //reduces all the swervemodule state speeds if one of them is over the max speed to make it so that none are over but all are still the same ratio
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve_module_states, Constants.dt.max_speed);

    //sets the module states
    for (SwerveModule module: this.dt) {
      module.set_desired_state(swerve_module_states[module.module_number]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //the order of the modules is front left front right back left then back right because that is the order the swerve map kinematics is defined in
    pose_estimator.update(get_yaw(), new SwerveModulePosition[] {this.dt[0].get_position(), this.dt[1].get_position(), this.dt[2].get_position(), this.dt[3].get_position()});
    //puts the robot position on the robot field and then puts the field on smartdashboard
    field.setRobotPose(pose_estimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Gyro Yaw", get_yaw().getDegrees());
    SmartDashboard.putNumber("Drive Encoder 0", this.dt[0].get_drive_encoder());
    SmartDashboard.putNumber("Drive Encoder 1", this.dt[1].get_drive_encoder());
    SmartDashboard.putNumber("Drive Encoder 2", this.dt[2].get_drive_encoder());
    SmartDashboard.putNumber("Drive Encoder 3", this.dt[3].get_drive_encoder());
  }
}
