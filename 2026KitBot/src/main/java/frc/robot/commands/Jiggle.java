// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class Jiggle extends Command {
    private final CANDriveSubsystem driveSubsystem;
    private final Timer timer = new Timer();
    private boolean goingForward = true;
    double speed;

    public Jiggle(CANDriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        this.driveSubsystem = driveSubsystem;

    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(0.292)) {
            goingForward = !goingForward;
        }

        if (goingForward) {
            speed = 0.5;
        }

        else {
            speed = -0.5;
        }

        driveSubsystem.driveArcade(speed, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(0, 0);
    }
}