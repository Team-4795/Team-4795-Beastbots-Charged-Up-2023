// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Telescope extends SubsystemBase {

private final CANSparkMax bob = new CANSparkMax(2, MotorType.kBrushless);

    public Telescope() {

    bob.restoreFactoryDefaults();
    bob.setIdleMode(IdleMode.kBrake);
    bob.burnFlash();
      
    }

    public void extend() {
      bob.set(0.25);
    }

    public void retract() {
      bob.set(-.25);
    }

    public void setSoftLimit(){
      bob.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      bob.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      bob.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0); //filler values for now
      bob.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0); //filler values for now

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
