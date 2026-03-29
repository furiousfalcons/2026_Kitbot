// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.FuelConstants.INTAKING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.INTAKING_INTAKE_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCHING_FEEDER_VOLTAGE;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE;
import frc.robot.subsystems.CANFuelSubsystem;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Unlaunch extends Command {

  /** Creates a new Intake. */

  CANFuelSubsystem fuelSubsystem;
  double startTime = 0;


  public Unlaunch(CANFuelSubsystem fuelSystem){
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
      // intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    // feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);
    
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
    startTime++;
    SmartDashboard.putNumber("bettertime", startTime);
    SmartDashboard.putNumber("a",fuelSubsystem.get_voltage());
    if(fuelSubsystem.get_voltage()<15){
        fuelSubsystem.setIntakeLauncherRoller(
            SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    startTime = Timer.getFPGATimestamp();
        }else{
            fuelSubsystem
        .setIntakeLauncherRoller(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
    fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
        }

  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
