package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;

public class AngleDriveCommand extends CommandBase{

    private DrivebaseSubsystem subsystem;
    private DoubleSupplier leftX, leftY, rightX, rightY;
    private double angle;
    private double[] angles = {0, 45, 90, 135, 180, 225, 270, 315};
    
    
    public AngleDriveCommand(DrivebaseSubsystem subsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY) {
        
        this.subsystem = subsystem;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        angle = subsystem.getGyroRotation().getDegrees();
    }

    @Override
    public void execute() {
        double lX = leftX.getAsDouble();
        double lY = leftY.getAsDouble();
        double rX = rightX.getAsDouble();
        double rY = rightY.getAsDouble();

        if(Util.vectorMagnitude(rX, rY) > 0.7) {
            angle = Util.angleSnap(Util.vectorToAngle(rX, rY), angles);
        }
        subsystem.driveAngle(lY, lX, angle);
    }

    @Override 
    public void end(boolean interrupted) {
        subsystem.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Util.epsilonZero(Util.relativeAngularDifference(subsystem.getGyroRotation(), angle), 1) 
        && Util.vectorMagnitude(rightX.getAsDouble(), rightY.getAsDouble()) <= 0.7 
        && Util.epsilonEquals(subsystem.getRotVelocity(), 0, 10);
    }

}
