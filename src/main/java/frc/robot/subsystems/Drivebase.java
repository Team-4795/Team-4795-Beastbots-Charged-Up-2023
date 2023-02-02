// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  private final PWMTalonSRX leftSide = new PWMTalonSRX(0);
  private final PWMTalonSRX rightSide = new PWMTalonSRX(1);

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftSide, rightSide);

  public Drivebase() {
    leftSide.setInverted(true);

  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }


  @Override
  public void periodic() {
    
  }
}