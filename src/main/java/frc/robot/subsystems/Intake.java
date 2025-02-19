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
    //creates the solenoid for the intake going in and out
    //The pneumatics module type is REVPH(REV Pneumatics Hub is the pneumatics controller made by REV Robotics)
    //the channels are plugged into 0 and 1, but which is which needs to be checked
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    //The intake motor is a NEO Vortex which is beinc controlled by a spark flex controller
    //neo vortex is brushless
    intake_motor = new SparkFlex(Constants.intake.motor_id, MotorType.kBrushless);

    //creates the spark flex configs for the intake motor
    //there are two configs because we will need two different current limits
    //the current limit blocks how many electrons can go through the motor at one time, which effectively controls the motors resistance
    //when holding a ball we will want a low current limit so that we can spin in and hold the ball without destroying it
    hold_config = new SparkFlexConfig();
    //when intaking and shooting we will want a higher current limit so that we can pull the ball in
    intake_config = new SparkFlexConfig();

    //configures the spark flex motor controller
    intake_config
      //sets the idle mode to brake
      .idleMode(IdleMode.kBrake)
      //sets the motor to not be inverted
      .inverted(false)
      //sets the current limit to the intake current limit from Constants
      .smartCurrentLimit(Constants.intake.intake_current_limit)
      //will scale all .run() on the motor controller as if the battery has full charge
      .voltageCompensation(12);

    //configures the spark flex motor controller
    hold_config
      //sets the idle mode to brake
      .idleMode(IdleMode.kBrake)
      //sets the motor to not be inverted
      .inverted(false)
      //sets the current limit to the hold current limit from Constants
      .smartCurrentLimit(Constants.intake.hold_current_limit)
      //will scale all .run() on the motor controller as if the battery has full charge
      .voltageCompensation(12);

    //configures the spark flex motor controller with the intake config
    intake_motor.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //sets the roller speed of the intake through a parameter
  public void run(double speed) {
    intake_motor.set(speed);
  }

  //changes the configuration of the spark flex motor controller based on if the motor is going to be holding algae or intaking algae
  public void configure(Constants.intake.state state) {
    //switch is a quick if else statement
    //which ever case is true will defnine which config is used
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
