package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto1 extends SequentialCommandGroup {

    public Auto1(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double angle1, double angle2, double distance, double speed){
        addCommands(new TurnToAngle(driveSubsystem, angle1).withTimeout(2), //turn to straight path toward depot
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), //straight path to depot
        new TurnToAngle(driveSubsystem, angle2).withTimeout(2), //align parallely with depot
        new IntakeAndReverse(ballSubsystem, driveSubsystem).withTimeout(4), // intaking while backing into the depot
        new DriveDistance(driveSubsystem, true, 0.1).withTimeout(4), // back out of the depot
        new TurnToAngle(driveSubsystem, -angle2).withTimeout(2), // turn to straight path to hub
        new DriveDistance(driveSubsystem, true, speed).withTimeout(distance/ speed), // straight path to hub
        new TurnToAngle(driveSubsystem, -angle1).withTimeout(2), // align with hub
        new Launch(ballSubsystem).withTimeout(5) // shooot
        );
    }

}
