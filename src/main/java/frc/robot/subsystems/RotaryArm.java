package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotaryArm extends SubsystemBase {
    private final CANSparkMax BaseMotor = new CANSparkMax(4, MotorType.kBrushless);
    
    public void LiftArm(){
        BaseMotor.set(.1);
      }
      public void LowerArm(){
        BaseMotor.set(-.1);
      }
}
