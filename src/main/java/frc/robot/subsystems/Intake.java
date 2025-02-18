// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  DoubleSolenoid solenoid;
  SparkFlex intake_motor;
  SparkFlexConfig hold_config;
  SparkFlexConfig intake_config;
  public Intake() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    intake_motor = new SparkFlex(Constants.intake.motor_id, MotorType.kBrushless);

    hold_config = new SparkFlexConfig();
    intake_config = new SparkFlexConfig();

    intake_config
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(Constants.intake.intake_current_limit);

    hold_config
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(Constants.intake.hold_current_limit);

    intake_motor.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //sets the roller speed of the intake through a parameter
  public void run(double speed) {
    intake_motor.set(speed);
  }

  //changes the configuration of the spark flex motor controller based on if the motor is going to be holding algae or intaking algae
  public void configure(Constants.intake.state state) {
    switch (state) {
      case HOLD:
        intake_motor.configure(hold_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        break;
      case INTAKE:
        intake_motor.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        break;
    }
  }

  //pushes pistons out
  public void extend() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  //pulls pistons in
  public void retract() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
