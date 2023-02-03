// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivebaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_subsystem;
  private DoubleSupplier speedY, speedX, rotX;

  public DrivebaseCommand(DrivebaseSubsystem subsystem, DoubleSupplier speedY, DoubleSupplier speedX, DoubleSupplier rotX) {
    
    this.speedY = speedY;
    this.speedX = speedX;
    this.rotX = rotX;

    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.drive(speedY.getAsDouble(), speedX.getAsDouble(), rotX.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
