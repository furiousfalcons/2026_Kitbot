// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import java.lang.Math;
import java.security.GeneralSecurityException;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;

import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;
  private final DifferentialDriveKinematics m_Kinematics;
  private final ADIS16448_IMU m_gyro;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final double positionConversionFactor;
  private final DifferentialDriveOdometry m_odometry;
  private Pose2d m_pose;
  private final EncoderConfig encoderConfig = new EncoderConfig();
  private SparkMaxConfig allConfigs = new SparkMaxConfig();
  private RobotConfig config;
  SparkMaxConfig motorConfig;
  //VisionSubsystem visionSubsystem = new VisionSubsystem();
  PIDController angController;
  public double speedMultiplier;


  private final DifferentialDrivePoseEstimator m_PoseEstimator;



  public CANDriveSubsystem() {
    speedMultiplier = 1;
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    motorConfig = new SparkMaxConfig();
    motorConfig.voltageCompensation(12);
    motorConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    motorConfig.follow(leftLeader);
    leftFollower.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig.follow(rightLeader);
    rightFollower.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    motorConfig.disableFollowerMode();
    rightLeader.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    motorConfig.inverted(true);
    leftLeader.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    ////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    m_Kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));
    m_gyro = new ADIS16448_IMU();
    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();
    positionConversionFactor = (Math.PI * WHEEL_DIAMETER)/GEAR_REDUCTION;
    motorConfig.encoder.positionConversionFactor(positionConversionFactor);  
  
    m_odometry = new DifferentialDriveOdometry(
      new Rotation2d(m_gyro.getAngle()), 
      Units.inchesToMeters(m_leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION), 
      Units.inchesToMeters(m_rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION),
      new Pose2d());

      // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }



      m_PoseEstimator = new DifferentialDrivePoseEstimator(
      m_Kinematics, 
      new Rotation2d(m_gyro.getAngle()), 
      Units.inchesToMeters(m_leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION), 
      Units.inchesToMeters(m_rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION),
      new Pose2d(),
      VecBuilder.fill(0.05,0.05,Units.degreesToRadians(5)),
      VecBuilder.fill(0.5,0.5,Units.degreesToRadians(30)));
      m_PoseEstimator.update(      new Rotation2d(m_gyro.getAngle()), 
      m_leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION, 
      m_rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION);


         // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }



  @Override
  public void periodic() {
    m_pose = m_odometry.update(new Rotation2d(m_gyro.getAngle()),
      Units.inchesToMeters(m_leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION), 
      Units.inchesToMeters(m_rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION));
      //m_PoseEstimator.addVisionMeasurement(visionSubsystem.getVisionMeasurement().getFirst().toPose2d(), 
          //visionSubsystem.getVisionMeasurement().getSecond().doubleValue());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void resetPose() {
    m_pose = new Pose2d();
  }

  public void resetPose(Pose2d newPose) {
    m_odometry.resetPosition(new Rotation2d(m_gyro.getAngle()),
      Units.inchesToMeters(m_leftEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION), 
      Units.inchesToMeters(m_rightEncoder.getPosition()*Math.PI*WHEEL_DIAMETER/GEAR_REDUCTION),
      newPose);

    m_pose = newPose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
      return m_Kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),m_rightEncoder.getVelocity()));
    }


  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    driveArcade(targetSpeeds.vxMetersPerSecond, targetSpeeds.omegaRadiansPerSecond * 0.02 + m_pose.getRotation().getRadians());
  }

  public DifferentialDrive getDrive(){
    return drive;
  }

  public ADIS16448_IMU getGyro(){
    return m_gyro;
  }
  
  
  }

