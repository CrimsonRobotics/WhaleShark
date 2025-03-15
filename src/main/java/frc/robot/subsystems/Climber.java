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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax motor;
  SparkMaxConfig config;
  double speed;
  PIDController pid;
  RelativeEncoder encoder;
  ShuffleboardTab climber_tab;
  GenericEntry climber_position;
  SparkMax motor_2;

  public Climber() {
    /**creates the climb motor using the motor id from Constants. */
    motor = new SparkMax(Constants.climber.motor_id, MotorType.kBrushless);
    motor_2 = new SparkMax(Constants.climber.motor_2_id, MotorType.kBrushless);


    /**gets the encoder from the motor_2 */
    encoder = motor.getEncoder();
    encoder.setPosition(0);

    /**creates and defines the spark max config */
    config = new SparkMaxConfig();

    config
      /**idle mode is brake */
      .idleMode(IdleMode.kBrake)
      /**motor is not inverted */
      .inverted(false)
      .voltageCompensation(12)
      .follow(motor);
    config.encoder
      .positionConversionFactor(Constants.climber.position_conversion_factor);

    /**
     * configures the motor with the config
     * when this is run all old settings on spark max are reset to default
     * then config is applied
     * persist means that if robot is power cycled, the settings will remain
     */
    
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor_2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /**this creates the pidcontroller that is used when putting the climber in the ready position */
    pid = new PIDController(Constants.climber.kp, Constants.climber.ki, Constants.climber.kd);

    /**this creates the shuffleboard tab for outputing climber data onto shuffleboard */
    climber_tab = Shuffleboard.getTab("Climber");
    /**puts the climber current position onto the shuffleboard tab as "Position" */
    climber_position = climber_tab.add("Position", 0).getEntry();
  }

  /**
   * @return this will return the current position of the climb motor in rotations
   * it is used for putting the climber in the ready position
   */
  public double get_position() {
    return encoder.getPosition();
    
  }


  /**
   * this will have the climber move to a specific position
   * it is used for putting the climber in the ready position
   * @param position the desired position in rotations to move the climber to
   */
  public void run_to_position(double position) {
    /**this calculates the voltage that needs to be applied using the pid controller, the current position, and the desired position(which is passed in as a parameter) */
    speed = MathUtil.clamp(pid.calculate(get_position(), position), -Constants.climber.max_speed, Constants.climber.max_speed);
    /**sets the voltage to the motor */
    motor.set(speed);
  }

  /**
   * this will just run the motor at a constants voltage
   * it is used for actually climbing at the end of the match, after the climber is put in ready position
   * @param speed speed at which to spin the motor
   */
  public void run(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /**this will put the current position of the climber onto shuffleboard continually */
    climber_position.setDouble(get_position());
  }
}
