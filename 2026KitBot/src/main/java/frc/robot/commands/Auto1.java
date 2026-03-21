package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto1 extends SequentialCommandGroup {

    public Auto1(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double angle1, double angle2, double distance, double speed){
        addCommands( new DriveDistance(driveSubsystem, false, speed). withTimeout(0.305/speed),
            new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(8), // shoot
        new DriveDistance(driveSubsystem, true, speed). withTimeout(0.305/speed),
        new TurnToAngle(driveSubsystem, angle1).withTimeout(1), //turn to straight path toward depot
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), //straight path to depot
        new TurnToAngle(driveSubsystem, angle2).withTimeout(1), //align parallely with depot
        new IntakeAndReverse(ballSubsystem, driveSubsystem).withTimeout(4), // intaking while backing into the depot
        new DriveDistance(driveSubsystem, true, 0.1).withTimeout(3), // back out of the depot
        new TurnToAngle(driveSubsystem, -angle2).withTimeout(1), // turn to straight path to hub
        new DriveDistance(driveSubsystem, true, speed).withTimeout(distance/ speed), // straight path to hub
        new TurnToAngle(driveSubsystem, -angle1).withTimeout(1), // align with hub
        new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(5) // shooot
        );
    }

}
