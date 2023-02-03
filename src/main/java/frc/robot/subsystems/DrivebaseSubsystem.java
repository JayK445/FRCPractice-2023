// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.TalonMecanumDrive;
import frc.util.Util;

public class DrivebaseSubsystem extends SubsystemBase {
  
  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private TalonMecanumDrive m_mecanumDrive;
  private ADXRS450_Gyro gyro;
  private double speedY, speedX, rotX, targetAngle, rotVelocity, lastAngle, lastTime;
  private PIDController controller;

  private enum DriveMode {
    DEFAULT, ANGLE
  }

  private DriveMode mode = DriveMode.DEFAULT;

  public DrivebaseSubsystem() {

    m_frontLeft = new TalonSRX(3);
    m_frontRight = new TalonSRX(1);
    m_backLeft = new TalonSRX(6);
    m_backRight = new TalonSRX(7);

    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);

    m_mecanumDrive = new TalonMecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);

    gyro = new ADXRS450_Gyro(Port.kMXP);

    controller = new PIDController(0.003, 0, 0.0005);
    controller.setSetpoint(0);
    controller.setTolerance(1);

    zeroGyro();
  }

  public void drive(double speedY, double speedX, double rotX) {
    this.speedY = speedY;
    this.speedX = speedX;
    this.rotX = rotX;
    mode = DriveMode.DEFAULT;
  }

  public void driveAngle(double speedY, double speedX, double targetAngle) {
    this.speedY = speedY;
    this.speedX = speedX;
    this.targetAngle = targetAngle;
    if(mode != DriveMode.ANGLE) controller.reset();
    mode = DriveMode.ANGLE;
  }

  private void updateRotVelocity() {
    double time = Timer.getFPGATimestamp();
    double angle = getGyroRotation().getDegrees();
    rotVelocity = (angle - lastAngle) / (time - lastTime);
    lastTime = time;
    lastAngle = angle;
  }

  public double getRotVelocity() {
    return rotVelocity;
  }

  public void zeroGyro() {
    gyro.reset();
  }
  
  public Rotation2d getGyroRotation() {

    double angle = Util.normalizeDegrees(gyro.getAngle());

    return Rotation2d.fromDegrees(angle);
  }

  public DriveMode getMode() {
    return mode;
  }

  public void switchMode() {
    switch(mode) {
      case DEFAULT : mode = DriveMode.ANGLE; break;
      case ANGLE : mode = DriveMode.DEFAULT; break;
    }
  }

  private void defaultPeriodic() {
    m_mecanumDrive.driveCartesian(speedY, speedX, rotX);
  }

  private void anglePeriodic() {
    double angleDifference = Util.relativeAngularDifference(getGyroRotation(), targetAngle);
    double rotationValue = controller.calculate(angleDifference);

    rotationValue = MathUtil.clamp(rotationValue, -1, 1);

    m_mecanumDrive.driveCartesian(speedY, speedX, rotationValue);

  }

  @Override
  public void periodic() {
    
    updateRotVelocity();

    DriveMode currentMode = getMode();
    switch(currentMode) {
      case DEFAULT : defaultPeriodic();
      case ANGLE : anglePeriodic();
    }
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
