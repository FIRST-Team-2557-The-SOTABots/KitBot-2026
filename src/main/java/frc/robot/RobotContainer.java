// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final FuelSubsystem fuelSubsystem = new FuelSubsystem();
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    
    public RobotContainer() {configureButtonBindings();
        driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem
        .drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true),
        driveSubsystem));
        
        boolean isCompetition = false;
        
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream
        );
            SmartDashboard.putData("Auto Chooser", autoChooser);
        }
        
    private void configureButtonBindings() {
        m_driverController.a().onTrue(new RunCommand(() -> fuelSubsystem.setVoltage(
FuelConstants.INTAKING_INTAKE_VOLTAGE, FuelConstants.INTAKING_FEEDER_VOLTAGE ), fuelSubsystem))
    .onFalse(new RunCommand(() -> fuelSubsystem.setVoltage(0,0), fuelSubsystem));
        
    
        m_driverController.rightBumper().onTrue(new RunCommand(() -> fuelSubsystem.setVoltage(
-FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE, -FuelConstants.LAUNCHING_FEEDER_VOLTAGE), fuelSubsystem))
    .onFalse(new RunCommand(() -> fuelSubsystem.setVoltage(0,0), fuelSubsystem));
    
    
        m_driverController.leftBumper().onTrue(new RunCommand(() -> fuelSubsystem.setVoltage(-FuelConstants.INTAKING_INTAKE_VOLTAGE, 
-FuelConstants.INTAKING_FEEDER_VOLTAGE ), fuelSubsystem))
        .onFalse(new RunCommand(() -> fuelSubsystem.setVoltage(0,0), fuelSubsystem));
        
        m_driverController.start().onTrue(Commands.runOnce(() -> driveSubsystem.zeroHeading()));
    
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
