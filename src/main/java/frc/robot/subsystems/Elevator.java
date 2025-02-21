// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkMax r_motor;
  SparkMax l_motor;
  SparkMaxConfig l_motor_config;
  SparkMaxConfig r_motor_config;
  RelativeEncoder encoder;
  PIDController pid;
  double voltage;
  ShuffleboardTab elevator_tab;
  GenericEntry elevator_position;
  GenericEntry elevator_velocity;
  public Elevator() {
    //Creates spark max motor controllers
    r_motor = new SparkMax(Constants.elevator.r_motor_id, MotorType.kBrushless);
    l_motor = new SparkMax(Constants.elevator.l_motor_id, MotorType.kBrushless);

    //Gets the encoder that is plugged into the spark max controller
    //It isnt the encoder built into the neo motor
    encoder = r_motor.getEncoder();
    encoder.setPosition(0);

    //Creates spark max motor controller configurations
    r_motor_config = new SparkMaxConfig();
    l_motor_config = new SparkMaxConfig();

    //sets the right motor configf
    r_motor_config
      //idle mode is brake
      .idleMode(IdleMode.kCoast)
      //motor is not inverted
      .inverted(true);
    r_motor_config.encoder
      //sets the position conversion factor to the elevator position conversion factor from Constants
      .positionConversionFactor(1);
    
    //sets the left motor config
    l_motor_config
      //idle mode is brake
      .idleMode(IdleMode.kCoast)
      //motor is not inverted
      .inverted(true);
    l_motor_config.encoder
      //sets the position conversion factor to the elevator position conversion factor from Constants
      .positionConversionFactor(Constants.elevator.position_conversion_factor);
    
    //configures the motor controllers with the config s
    //when this is run all old settings on spark max are reset to default
    //then config is applied
    //persist means that if robot is power cycled, the settings will remain
    r_motor.configure(r_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    l_motor.configure(l_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    //this creates the pidcontroller that is used when putting the climber in the ready position
    pid = new PIDController(Constants.elevator.kp, Constants.elevator.ki, Constants.elevator.kd);

    //this creates the shuffleboard tab for outputing elevator data onto shuffleboard
    elevator_tab = Shuffleboard.getTab("Elevator");
    //puts the elevator current position onto the shuffleboard tab as "ELevator Position"
    elevator_position = elevator_tab.add("Elevator Position", 0).getEntry();
    //puts the elevator current velocity onto the shuffleboard tab as "Elevator Velocity"
    elevator_velocity = elevator_tab.add("Elevator Velocity", 0).getEntry();
  }

  //this will return the current position of the elevator motor
  //it is used for putting the elevator into various intake and scoring positions
  public double get_position() {
    return encoder.getPosition();
  }

  public double get_velocity() {
    return encoder.getVelocity();
  }

  public void run(double speed) {
    r_motor.set(speed);
    l_motor.set(speed);
  }

  //this will have the elevator move to a specific position
  //it is used for putting the elevator into various intake and scoring positions
  public void run_to_position(double position) {
    //this calculates the voltage that needs to be applied using the pid controller, the current position, and the desired position(which is passed in as a parameter)
    voltage = pid.calculate(get_position(), position);
    //sets the voltage to the motors
    r_motor.setVoltage(voltage);
    l_motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //this will put the current position of the climber onto shuffleboard continually
    elevator_position.setDouble(get_position());
    ////this will put the current velocity of the climber onto shuffleboard continually
    elevator_velocity.setDouble(get_velocity());
    SmartDashboard.putNumber("Elevator Position", get_position());
  }
}
