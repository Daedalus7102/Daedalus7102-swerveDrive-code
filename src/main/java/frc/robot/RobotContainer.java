// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveCommands.DriveCommand;
import frc.robot.subsystems.Drive.Swerve;

public class RobotContainer {

  // Subsystems
  public Swerve swerve = new Swerve();

  // Controllers
  public static final CommandPS5Controller driverController = new CommandPS5Controller(0);
  public static final CommandPS5Controller operatorController = new CommandPS5Controller(1);
  private SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ----------- Driver Controller -----------
    swerve.setDefaultCommand(
      new DriveCommand(
        swerve,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        false,
        SwerveConstants.chassisHighMaxOutput
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        swerve,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        true,
        SwerveConstants.chassisLowMaxOutput
      )
    );

    // ------------ Driver Controller ------------
    driverController.triangle().whileTrue(new InstantCommand(() -> swerve.zeroHeading()));

    // ----------- Operator Controller -----------

  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return autoChooser.getSelected();
  }

  public Swerve getChasisSubsystem() {
    return swerve;
  }
}