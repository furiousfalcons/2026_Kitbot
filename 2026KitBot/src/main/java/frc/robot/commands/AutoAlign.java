package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
public class AutoAlign extends SequentialCommandGroup {

    public AutoAlign(VisionSubsystem visionSubsystem, CANDriveSubsystem driveSubsystem) {
        double angle = visionSubsystem.getAngleToAlign();
        addCommands(new TurnToAngle(driveSubsystem, angle));
    }

}
