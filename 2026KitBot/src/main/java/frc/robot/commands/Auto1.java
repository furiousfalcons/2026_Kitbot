package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto1 extends SequentialCommandGroup {
    CANDriveSubsystem driveSubsystem;
    CANFuelSubsystem ballSubsystem;

    public Auto1(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double angle1, double angle2, double distance){
        addCommands(new AutoDrive(driveSubsystem, angle1).withTimeout(2),
        new DriveDistance(driveSubsystem, false).withTimeout(distance/ 0.5),
        new AutoDrive(driveSubsystem, angle2).withTimeout(2),
        new Intake(ballSubsystem).withTimeout(4),
        new AutoDrive(driveSubsystem, -angle2).withTimeout(2),
        new DriveDistance(driveSubsystem, true).withTimeout(2),
        new AutoDrive(driveSubsystem, angle2).withTimeout(2),
        new Launch(ballSubsystem).withTimeout(5)
        );
    }



}
