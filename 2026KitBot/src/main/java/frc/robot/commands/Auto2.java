package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto2 extends SequentialCommandGroup {

    public Auto2(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double angle1, double angle2, double distance, double speed){
        addCommands(new TurnToAngle(driveSubsystem, angle1).withTimeout(2), //turn to straight path toward outpost
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), //straight path to outpost
        new TurnToAngle(driveSubsystem, angle2).withTimeout(2), //align parallely with outpost
        new DriveDistance(driveSubsystem, true, 0.1).withTimeout(6), // back into the outpost
        new DriveDistance(driveSubsystem, true, 0).withTimeout(4), // wait while intaking
        new DriveDistance(driveSubsystem, false, 0.1).withTimeout(6), // back out of the outpost
        new TurnToAngle(driveSubsystem, -angle2).withTimeout(2), // turn to straight path to hub
        new DriveDistance(driveSubsystem, true, speed).withTimeout(distance/ speed), // straight path to hub
        new TurnToAngle(driveSubsystem, angle1).withTimeout(2), // align with hub
        new Launch(ballSubsystem, driveSubsystem).withTimeout(5) // shooot
        );
    }

}
