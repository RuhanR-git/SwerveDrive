// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoyStickCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
  
  public RobotContainer() 
  {
    configureBindings();
    swerveSubsystem.setDefaultCommand
    ( 
      new SwerveJoyStickCommand
      (
        swerveSubsystem, 
        () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      )
    );
  }

    private void configureBindings()
    {
      new JoystickButton(driverJoystick, 2).onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
    }

    public Command getAutonomousCommand() {
      return null;  
    }
}