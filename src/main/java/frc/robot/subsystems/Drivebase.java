// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax.IdleMode;

//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  /*
  private final CANSparkMax leftLeader = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax leftFollowerTwo = new CANSparkMax(4, MotorType.kBrushless);

  private final CANSparkMax rightLeader = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax rightFollowerTwo = new CANSparkMax(7, MotorType.kBrushless);

  */



private final PWMTalonSRX leftLeader = new PWMTalonSRX(2); 
private final PWMTalonSRX leftFollower = new PWMTalonSRX(3); 
private final PWMTalonSRX leftFollowerTwo = new PWMTalonSRX(4); 

private final PWMTalonSRX rightLeader = new PWMTalonSRX(5); 
private final PWMTalonSRX rightFollower = new PWMTalonSRX(6); 
private final PWMTalonSRX rightFollowerTwo = new PWMTalonSRX(7); 

private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader); 

  public Drivebase() {
   
   

   //leftFollower.follow(leftLeader);
    //leftFollowerTwo.follow(leftLeader);
    //rightFollowerTwo.follow(rightLeader);
   // rightFollower.follow(rightLeader);

    //leftLeader.setIdleMode(IdleMode.kBrake);
    //leftFollower.setIdleMode(IdleMode.kBrake);
    //rightLeader.setIdleMode(IdleMode.kBrake);
    //rightFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    leftFollowerTwo.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
    rightFollowerTwo.setInverted(false);

  }

  public void set_left(double speed){
    leftFollower.set(speed);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }


  @Override
  public void periodic() {
    
  }
}