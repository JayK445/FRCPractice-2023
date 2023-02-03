// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AngleDriveCommand;
import frc.robot.commands.DrivebaseCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private XboxController controller_1 = new XboxController(0);

  private DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    m_drivebaseSubsystem.setDefaultCommand(new DrivebaseCommand(m_drivebaseSubsystem, () -> (squareInputs(controller_1.getLeftY())), () -> (squareInputs(controller_1.getLeftX())), () -> (squareInputs(controller_1.getRightX()))));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new Button(controller_1::getRightStickButton).whileHeld(new AngleDriveCommand(m_drivebaseSubsystem, 
    controller_1::getLeftX, controller_1::getLeftY, controller_1::getRightX, controller_1::getRightX));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private static double squareInputs(double value) {
    
    value = Math.copySign(value*value, value);
    return value;
  }
}
