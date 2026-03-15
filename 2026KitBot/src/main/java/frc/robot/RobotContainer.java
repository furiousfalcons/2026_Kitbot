// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;
import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  double DepotY;
  double DepotX;
  double OutpostX;
  double OutpostY;
  double theta;
  double robotHeading;
  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", new ExampleAuto(driveSubsystem, fuelSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Pose2d currPose = visionSubsystem.getAutoPose();

    if (currPose.equals(new Pose2d())){ // so it doesn't crash out if it's not looking at an apriltag
    
        return null;
    }
    double currX = currPose.getX();
    double currY = currPose.getY();
    robotHeading = currPose.getRotation().getDegrees();

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      DepotX = 15.513272;
      DepotY = 2.116931;
      OutpostX = 15.780766;
      OutpostY = 7.40965675;
      if (robotHeading < 0) {
        robotHeading = (180 + robotHeading);
      } else {
        robotHeading = (robotHeading - 180);
      }
    }

    else {
      DepotX = 1.01600;
      DepotY = 6.0539155;
      OutpostX = 0.7366;
      OutpostY = 0.67230625;
      robotHeading = currPose.getRotation().getDegrees();
    }

    double depotTheta = Math.abs(Math.atan((currY - DepotY) / (currX - DepotX)) * 180 / Math.PI);
    double depotDistance = Math.pow(Math.pow((currY - DepotY), 2) + Math.pow((currX - DepotX), 2), 0.5);

    double outpostTheta = Math.abs(Math.atan((currY - OutpostY) / (currX - OutpostX)) * 180 / Math.PI);
    double outpostDistance = Math.pow(Math.pow((currY - OutpostY), 2) + Math.pow((currX - OutpostX), 2), 0.5);

    double angle1 = -1 * depotTheta - robotHeading;
    double angle2 = depotTheta;

    double angle3 = outpostTheta - robotHeading;
    double angle4 = 180 - outpostTheta;

    Command depotCommand = new Auto1(driveSubsystem, fuelSubsystem, angle1, angle2, depotDistance, 0.5);
    SmartDashboard.putNumber("Angle1", angle1);
    SmartDashboard.putNumber("Angle2", angle2);
    SmartDashboard.putNumber("Distance", depotDistance);
    driveSubsystem.getGyro().reset();


    Command outpostCommand = new Auto2(driveSubsystem, fuelSubsystem, angle3, angle4, outpostDistance, 0.5);
    // double angle1 = 45;
    // double angle2 = -45;

    // SmartDashboard.putNumber("angle", angle1);
    // autoChooser.addOption("Depot", depotCommand);
    // autoChooser.addOption("Outpost", outpostCommand);

    // return autoChooser.getSelected();
    return depotCommand;

  }
}
