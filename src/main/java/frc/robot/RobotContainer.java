// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoyStickCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final XboxController driverController = new XboxController(0);
  
  public RobotContainer() 
  {
    configureBindings();
    swerveSubsystem.setDefaultCommand( 
      new SwerveJoyStickCommand(
        swerveSubsystem, 
        () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      )
      );
  }

    private void configureBindings()
    {

    }

    public Command getAutonomousCommand() {
      return null;  
    }
}