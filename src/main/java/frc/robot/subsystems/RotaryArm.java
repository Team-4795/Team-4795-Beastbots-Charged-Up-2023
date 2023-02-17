package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotaryArm extends SubsystemBase {
    private final CANSparkMax BaseMotor = new CANSparkMax(7, MotorType.kBrushless);
    private RelativeEncoder m_encoder;

    public RotaryArm() {
      BaseMotor.restoreFactoryDefaults();
      m_encoder = BaseMotor.getEncoder();

      SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
      SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    }

    public void LowerArm(){
      BaseMotor.set(-0.1);
    }

    public void LiftArm(){
      BaseMotor.set(0.1);
    }
}