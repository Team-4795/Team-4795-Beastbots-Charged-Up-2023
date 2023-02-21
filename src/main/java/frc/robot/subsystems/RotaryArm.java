package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotaryArm extends SubsystemBase {
    private final CANSparkMax BaseMotor = new CANSparkMax(7, MotorType.kBrushless);
    private RelativeEncoder m_encoder;
    private final SparkMaxPIDController RotaryPID = new SparkMaxPIDController(kp, ki, kd);
    m_PIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    //BaseMotor.set(RotaryPID.calculate(encoder.getDistance(), setpoint));

    public RotaryArm() {
      BaseMotor.restoreFactoryDefaults();
      m_encoder = BaseMotor.getEncoder();

      SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
      SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
    }

    public void LowerArm(){
      BaseMotor.set(-0.4);
    }

    public void LiftArm(){
      BaseMotor.set(0.4);
    }

    public void stopArm(){
      BaseMotor.set(0);
    }

    public void setSoftLimits(){
      BaseMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      BaseMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      BaseMotor.setSoftLimit(SoftLimitDirection.kForward, 160);
      BaseMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    }
}