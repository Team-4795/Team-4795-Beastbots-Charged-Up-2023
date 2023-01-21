// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(0);
  private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(8);
  private final WPI_TalonSRX leftFollowerTwo = new WPI_TalonSRX(13);

  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(1);
  private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(10);
  private final WPI_TalonSRX rightFollowerTwo = new WPI_TalonSRX(7);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader); 
  private final DifferentialDriveOdometry odometry;
  private final AHRS gyro;
  private Pose2d currentGoal;

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_leftEncoder;
 
  public Drivebase() {
    leftFollower.follow(leftLeader);
    leftFollowerTwo.follow(leftLeader);
    rightFollower.follow(rightLeader);
    rightFollowerTwo.follow(rightLeader);

    gyro = new AHRS(SPI.Port.kMXP);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    leftFollowerTwo.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
    rightFollowerTwo.setInverted(false);

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    m_leftEncoder.setVelocityConversionFactor((1.0 / 60.0 / DrivebaseConstants.GEARING) * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI);
    m_rightEncoder.setVelocityConversionFactor((1.0 / 60.0 / DrivebaseConstants.GEARING) * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI);

    resetEncoders();

    leftLeader.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    leftFollower.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    leftFollowerTwo.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightFollower.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);
    rightFollowerTwo.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);


    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();

    double leftDistance = m_leftEncoder.getPosition() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI;
    double rightDistance = m_rightEncoder.getPosition() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI;

    odometry.resetPosition(gyro.getRotation2d(), leftDistance, rightDistance, pose);
  } 

  @Override
  public void periodic() {
    
  }
}