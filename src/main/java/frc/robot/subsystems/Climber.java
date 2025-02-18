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
    motor = new SparkMax(Constants.climber.motor_id, MotorType.kBrushless);

    encoder = motor.getEncoder();

    config = new SparkMaxConfig();
    config
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pid = new PIDController(Constants.climber.kp, Constants.climber.ki, Constants.climber.kd);
  }

  public double get_position() {
    return encoder.getPosition();
  }

  public void run_to_position(double position) {
    voltage = pid.calculate(get_position(), position);
    motor.setVoltage(voltage);
  }

  public void run(double speed) {
    motor.setVoltage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
