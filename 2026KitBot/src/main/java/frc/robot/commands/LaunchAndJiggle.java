package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class LaunchAndJiggle extends ParallelCommandGroup{
    public LaunchAndJiggle(CANDriveSubsystem driveSubsystem, CANFuelSubsystem canFuelSubsystem){
        addCommands(
            new Jiggle(driveSubsystem),
            new Launch(canFuelSubsystem)
    );
    }

}
