package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
//103.11 inches left 75.93 up 36.367 degree angle 104.0288 inches
// 0.17 into depot

//67.5
//32 inches back
//75.93
public class Auto3 extends SequentialCommandGroup {

    public Auto3(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double speed){
        double distance = Units.inchesToMeters(101.59);
        double angle1 = 48.363;
        addCommands(new DriveDistance(driveSubsystem, false, speed). withTimeout(0.5/speed),
        //new Launch(ballSubsystem).withTimeout(1),
        new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(8), // shoot
        new DriveDistance(driveSubsystem, false, speed). withTimeout(12*2.54/100/speed),
         new TurnToAngle(driveSubsystem, -angle1).withTimeout(1), //turn to straight path toward depot
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), //straight path to depot
        new TurnToAngle(driveSubsystem, angle1).withTimeout(1), //align parallely with depot
        new IntakeAndReverse(ballSubsystem, driveSubsystem).withTimeout(2), // intaking while backing into the depot
        new DriveDistance(driveSubsystem, true, 0.34).withTimeout(2), // back out of the depot
        new TurnToAngle(driveSubsystem, -angle1).withTimeout(1), // turn to straight path to hub
        new DriveDistance(driveSubsystem, true, speed).withTimeout(distance/ speed), // straight path to hub
        new TurnToAngle(driveSubsystem, angle1).withTimeout(1), // align with hub
        new DriveDistance(driveSubsystem, true, speed). withTimeout(12*2.54/100/speed),

        new LaunchAndJiggle(driveSubsystem, ballSubsystem).withTimeout(5) // shooot
        );
    }

}