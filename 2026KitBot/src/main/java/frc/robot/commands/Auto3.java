package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
//103.11 inches left 75.93 up 36.367 degree angle 128.05 inches
// 0.17 into depot
public class Auto3 extends SequentialCommandGroup {

    public Auto3(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double speed){
        double distance = 3.252;
        double angle1 = 36.367;
        addCommands(new DriveDistance(driveSubsystem, false, speed). withTimeout(0.305/speed),
        new Launch(ballSubsystem).withTimeout(1),
        new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(8), // shoot
        new DriveDistance(driveSubsystem, true, speed). withTimeout(0.305/speed),
         new TurnToAngle(driveSubsystem, -angle1).withTimeout(1), //turn to straight path toward depot
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), //straight path to depot
        new TurnToAngle(driveSubsystem, angle1).withTimeout(1), //align parallely with depot
        new IntakeAndReverse(ballSubsystem, driveSubsystem).withTimeout(2), // intaking while backing into the depot
        new DriveDistance(driveSubsystem, true, 0.34).withTimeout(2), // back out of the depot
        new TurnToAngle(driveSubsystem, -angle1).withTimeout(1), // turn to straight path to hub
        new DriveDistance(driveSubsystem, true, speed).withTimeout(distance/ speed), // straight path to hub
        new TurnToAngle(driveSubsystem, angle1).withTimeout(1), // align with hub
        new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(5) // shooot
        );
    }

}
