package frc.robot.commands;


import frc.robot.subsystems.CANFuelSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Intake;

public class WaitAndIntake extends SequentialCommandGroup {

    public WaitAndIntake(CANFuelSubsystem ballSubsystem) {
        addCommands(    Commands.waitSeconds(3)
, new Intake(ballSubsystem)
        );
    }


}