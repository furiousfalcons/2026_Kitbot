package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class Auto3 extends SequentialCommandGroup {

    public Auto3(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem, double angle1, double angle2, double distance, double speed){
        addCommands(new TurnToAngle(driveSubsystem, angle1).withTimeout(2), //turn to straight path toward depot
        new DriveDistance(driveSubsystem, false, speed).withTimeout(distance/ speed), 
    }

}
