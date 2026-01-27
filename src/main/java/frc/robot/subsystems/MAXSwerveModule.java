// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.robot.Configs;

public class MAXSwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_turningSpark;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_turningClosedLoopController;
  private final VelocityVoltage m_drivingVelocityRequest;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration uses a Kraken (TalonFX) for
   * driving and a NEO with SPARK MAX for turning with a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingTalonFX = new TalonFX(drivingCANId);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Configure the Kraken (TalonFX)
    TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
    
    // Configure PID values for velocity control
    drivingConfig.Slot0.kP = 0.3; // Adjust these values for your robot
    drivingConfig.Slot0.kI = 0.0;
    drivingConfig.Slot0.kD = 0.0;
    drivingConfig.Slot0.kV = 0.12; // Feedforward gain
    
    // Configure motor output
    drivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    drivingConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    // Apply configuration
    m_drivingTalonFX.getConfigurator().apply(drivingConfig);
    
    // Initialize velocity control request
    m_drivingVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Apply the turning motor configuration
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    
    // Reset the Kraken encoder position
    m_drivingTalonFX.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // Note: TalonFX velocity is in rotations per second, convert as needed based on your gear ratio
    return new SwerveModuleState(m_drivingTalonFX.getVelocity().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    // Note: TalonFX position is in rotations, convert as needed based on your gear ratio
    return new SwerveModulePosition(
        m_drivingTalonFX.getPosition().getValueAsDouble(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving TalonFX and turning SPARK towards their respective setpoints.
    // Note: You'll need to convert speedMetersPerSecond to rotations per second based on your wheel diameter and gear ratio
    m_drivingTalonFX.setControl(m_drivingVelocityRequest.withVelocity(correctedDesiredState.speedMetersPerSecond));
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalonFX.setPosition(0);
  }
}