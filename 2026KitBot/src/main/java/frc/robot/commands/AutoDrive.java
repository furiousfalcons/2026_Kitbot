// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  double xSpeed, zRotation;
  ADIS16470_IMU gyro;
  PIDController angController;
  DifferentialDrive drive;
  double angle;



  public AutoDrive(CANDriveSubsystem driveSystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    gyro = driveSystem.getGyro();
    drive = driveSubsystem.getDrive();
    this.angle = angle;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
      double output = angController.calculate(gyro.getAngle(), angle);
     double speed = 4*output;
     drive.tankDrive(speed, -speed);  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
