// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax motor;
  SparkMaxConfig config;
  double voltage;
  PIDController pid;
  RelativeEncoder encoder;
  public Climber() {
    //creates the climb motor using the motor id from Constants
    motor = new SparkMax(Constants.climber.motor_id, MotorType.kBrushless);

    //gets the encoder from the motor
    encoder = motor.getEncoder();

    //creates and defines the spark max config
    config = new SparkMaxConfig();
    config
      //idle mode is brake
      .idleMode(IdleMode.kBrake)
      //motor is not inverted
      .inverted(false);

    //configures the motor with the config 
    //when this is run all old settings on spark max are reset to default
    //tjen config is applied
    //persist means that if robot is power cycled, the settings will remain
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //this creates the pidcontroller that is used when putting the climber in the ready position
    pid = new PIDController(Constants.climber.kp, Constants.climber.ki, Constants.climber.kd);
  }

  //this will return the current position of the climb motor
  //it is used for putting the climber in the ready position
  public double get_position() {
    return encoder.getPosition();
  }

  //this will have the climber move to a specific position
  //it is used for putting the climber in the ready position
  public void run_to_position(double position) {
    //this calculates the voltage that needs to be applied using the pid controller, the current position, and the desired position(which is passed in as a parameter)
    voltage = pid.calculate(get_position(), position);
    //sets the voltage to the motor
    motor.setVoltage(voltage);
  }

  //this will just run the motor at a constants voltage
  //it is used for actually climbing at the end of the match, after the climber is put in ready position
  public void run(double speed) {
    motor.setVoltage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
