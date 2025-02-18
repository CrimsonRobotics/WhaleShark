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
  public Elevator() {
    //Creates spark max motor controllers
    r_motor = new SparkMax(Constants.elevator.r_motor_id, MotorType.kBrushless);
    l_motor = new SparkMax(Constants.elevator.l_motor_id, MotorType.kBrushless);

    encoder = r_motor.getAlternateEncoder();

    //Creates spark max motor controller configurations
    r_motor_config = new SparkMaxConfig();
    l_motor_config = new SparkMaxConfig();

    r_motor_config
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    l_motor_config
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    r_motor.configure(r_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    l_motor.configure(l_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid = new PIDController(Constants.elevator.kp, Constants.elevator.ki, Constants.elevator.kd);
  }

  public double get_position() {
    return encoder.getPosition();
  }

  public void run_to_position(double position) {
    voltage = pid.calculate(get_position(), position);
    r_motor.setVoltage(voltage);
    l_motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
