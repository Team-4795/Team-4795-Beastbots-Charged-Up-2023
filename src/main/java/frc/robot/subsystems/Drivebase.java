// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// hamburger.

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

    resetEncoders();

    rightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    leftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0.0, 0.0);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeader.getSelectedSensorVelocity(), rightLeader.getSelectedSensorVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    diffDrive.feed();
  }

  public void resetEncoders() {
    rightLeader.setSelectedSensorPosition(0);
    leftLeader.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();

    double leftDistance = leftLeader.getSelectedSensorPosition();
    double rightDistance = rightLeader.getSelectedSensorPosition();

    odometry.resetPosition(gyro.getRotation2d(), leftDistance, rightDistance, pose);
  } 

  @Override
  public void periodic() {
    
  }
}