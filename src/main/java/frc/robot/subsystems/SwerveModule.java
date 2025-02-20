// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class SwerveModule {
    public final int module_number;
    private final Rotation2d turn_offset;
    private final SparkMax drive_motor;
    private final SparkMax turn_motor;
    private final SparkMaxConfig drive_config;
    private final SparkMaxConfig turn_config;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;
    private final CANcoder best_turn_encoder;
    private SwerveModuleState current_state;
    private Rotation2d current_rotation;
    private Rotation2d desired_rotation;
    private PIDController drive_pid;
    private PIDController turn_pid;
    private double turn_speed;
    private double drive_speed;
    private MagnetSensorConfigs magnet_sensor_config;

    public SwerveModule(int module_number, int drive_motor_id, int turn_motor_id, int can_coder_id, Rotation2d turn_offset) {
        this.module_number = module_number;
        this.turn_offset = turn_offset;




        //can coder stuff new firmware
        this.best_turn_encoder = new CANcoder(can_coder_id);
        //can coder magnet sensor conifg
        this.magnet_sensor_config = new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5);
        this.best_turn_encoder.getConfigurator().apply(this.magnet_sensor_config);
        
        //drive motor stuff
        //drive motor controller
        this.drive_motor = new SparkMax(drive_motor_id, MotorType.kBrushless);
        //gets drive encoder from drive motor controller
        this.drive_encoder = this.drive_motor.getEncoder();

        //drive motor config
        this.drive_config = new SparkMaxConfig();
        this.drive_config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .voltageCompensation(12);
        this.drive_config.encoder
            .positionConversionFactor(Constants.dt.drive_position_conversion_factor)
            .velocityConversionFactor(Constants.dt.drive_velocity_conversion_factor);

        //configures motor controller with config data
        this.drive_motor.configure(this.drive_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //resets drive encoder
        this.drive_encoder.setPosition(0);

        //drive pid controller
        this.drive_pid = new PIDController(Constants.dt.drive_kp, Constants.dt.drive_ki, Constants.dt.drive_kd);

        //turn motor stuff
        //turn motor controller
        this.turn_motor = new SparkMax(turn_motor_id, MotorType.kBrushless);
        //gets turn encoder from turn motor controller
        this.turn_encoder = this.turn_motor.getEncoder();

        //turn motor config
        this.turn_config = new SparkMaxConfig();
        this.turn_config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20)
            .voltageCompensation(12);
        this.turn_config.encoder
            .positionConversionFactor(Constants.dt.turn_position_conversion_factor);
        this.turn_config.signals
            .externalOrAltEncoderPosition(500);
        
        //configures turn motor controller with turn config data
        this.turn_motor.configure(this.turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //turn pid controller
        this.turn_pid = new PIDController(Constants.dt.turn_kp, Constants.dt.turn_ki, Constants.dt.turn_kd);
    }

    //resets turn encoders to cancoder offsets
    public void reset_encoder() {
        this.turn_encoder.setPosition(this.turn_offset.getDegrees());
    }

    public double get_drive_encoder() {
        return this.drive_encoder.getPosition();
    }

    //returns can coder value
    public Rotation2d get_can_coder() {
        return Rotation2d.fromDegrees(this.best_turn_encoder.getAbsolutePosition().getValue().magnitude());
    }

    //returns the current state of the module
    public SwerveModuleState get_state() {
        return new SwerveModuleState( this.drive_encoder.getVelocity(), get_can_coder());
    }

    //returns the current position of the module
    public SwerveModulePosition get_position() {
        return new SwerveModulePosition(this.drive_encoder.getPosition(),get_can_coder().minus(this.turn_offset));
    }

    //sets the swerve modules into their desired states using speed and angles
    public void set_desired_state(SwerveModuleState desired_state) {
        current_state = this.get_state();
        current_rotation = current_state.angle.minus(this.turn_offset).plus(new Rotation2d());
        
        //optimize the angle used in desired state to make sure itdoes not spin more than 90 degrees
        desired_state.optimize(current_rotation);

        //scale speed by cosine of angle error, which scales down movemment perpendicular to desired direction of travel which happens when modules change directions
        desired_state.cosineScale(current_rotation);

        desired_rotation = desired_state.angle.plus(new Rotation2d());

        double diff = desired_rotation.getDegrees() - current_rotation.getDegrees();
        
        if (Math.abs(diff) < 1) {
            turn_speed = 0;
        } else {
            turn_speed = this.turn_pid.calculate(diff, 0);
        }

        SmartDashboard.putNumber("Can_coder from module" + this.module_number, this.get_can_coder().getDegrees());
        SmartDashboard.putNumber("Difference" + this.module_number, diff);

        //drive_speed = this.drive_pid.calculate(drive_encoder.getVelocity(), desired_state.speedMetersPerSecond);
        drive_speed = desired_state.speedMetersPerSecond / Constants.dt.max_speed;

        this.turn_motor.set(turn_speed);
        this.drive_motor.set(drive_speed);
    }
}
