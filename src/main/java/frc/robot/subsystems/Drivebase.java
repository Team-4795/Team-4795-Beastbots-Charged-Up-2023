// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
``
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
  private final PWMTalonSRX leftLeader = new PWMTalonSRX(0); 
  private final PWMTalonSRX leftFollower = new PWMTalonSRX(8); 
  private final PWMTalonSRX leftFollowerTwo = new PWMTalonSRX(13); 

  private final PWMTalonSRX rightLeader = new PWMTalonSRX(1); 
  private final PWMTalonSRX rightFollower = new PWMTalonSRX(10); 
  private final PWMTalonSRX rightFollowerTwo = new PWMTalonSRX(7); 

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader); 
  private final DifferentialDriveOdometry  odometry;
  private final AHRS gyro;
  private Pose2d currentGoal;

  private final Encoder m_leftEncoder = new Encoder( );
  private final Encoder m_rightEncoder = new Encoder( );
 
  public Drivebase() {
    gyro = new AHRS(SPI.Port.kMXP);

    //m_leftEncoder = leftLeader.getEncoder();
    //m_rightEncoder = rightLeader.getEncoder();

    leftLeader.setInverted(true);
    //leftFollower.setInverted(true);
    //leftFollowerTwo.setInverted(true);
    rightLeader.setInverted(false);
    //rightFollower.setInverted(false);
    //rightFollowerTwo.setInverted(false);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }

  /* 
  public void spd(){
    leftFollower.set(leftLeader.get());
    rightFollower.set(rightLeader.get());
    leftFollowerTwo.set(leftLeader.get());
    rightFollowerTwo.set(rightLeader.get());
  } */

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
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    currentGoal = null;
    odometry.resetPosition(pose, gyro.getRotation2d());
  } 

  @Override
  public void periodic() {
    
  }
}