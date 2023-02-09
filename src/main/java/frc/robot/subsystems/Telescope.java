// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Telescope extends SubsystemBase {

private final CANSparkMax Bob = new CANSparkMax(2, MotorType.kBrushless);


    public Telescope() {

    Bob.restoreFactoryDefaults();
    Bob.setIdleMode(IdleMode.kBrake);
    Bob.burnFlash();
      
    }

    public void extend() {
      Bob.set(0.25);
    }

    public void retract() {
      Bob.set(-.25);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
