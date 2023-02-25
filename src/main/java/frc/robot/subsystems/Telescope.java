// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Telescope extends SubsystemBase {

private final CANSparkMax bob = new CANSparkMax(8, MotorType.kBrushless);
public final AbsoluteEncoder TelescopeEncoder;


    public Telescope() {
      TelescopeEncoder = bob.getAbsoluteEncoder(Type.kDutyCycle);
      TelescopeEncoder.setPositionConversionFactor(1);

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

    public void start() {
      bob.set(0.25);
    }

    public void moveTelescopeArm(double speed){
      
      if (TelescopeEncoder.getPosition()>2 && speed>0)
      {

      bob.set(0);
      }

      if (TelescopeEncoder.getPosition()<1 && speed<0)
      {

      bob.set(0);
      }
      
      else {
        bob.set(speed);
      }
    }

    public void stop(){
      bob.set(0);
    }

    public void setSoftLimit(){
      bob.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
      bob.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
      bob.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 80); //54 holes (roughly), 12 tooth sprocket, 28:1 gear reduction = 108(?)
      bob.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 30); 

    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }
}
