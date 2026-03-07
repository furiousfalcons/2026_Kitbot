package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveDistance extends Command{
    CANDriveSubsystem driveSubsystem;
    boolean forward;
    public DriveDistance(CANDriveSubsystem driveSystem, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.forward = forward;
    //controller = driverController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    double speed;
    if (forward){
            speed = 0.5;
    }
    else{
        speed = -0.5;
    }
    driveSubsystem.driveArcade(speed, 0);
    //SmartDashboard.putNumber("Angle", gyro.getAngle());
  }

  public void periodic(){
    //SmartDashboard.putNumber("Angle", gyro.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

