package frc.robot.subsystems;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
//import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

//import frc.robot.Constants;
//import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RotaryConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotaryArm extends SubsystemBase {
    private final CANSparkMax BaseMotor = new CANSparkMax(7, MotorType.kBrushless);
    private RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    public double setpoint; 

    //BaseMotor.set(RotaryPID.calculate(encoder.getDistance(), setpoint));
    private final SparkMaxPIDController RotaryPID;

    public RotaryArm() {
      RotaryPID = BaseMotor.getPIDController();
      absoluteEncoder = BaseMotor.getAbsoluteEncoder(Type.kDutyCycle);
      this.setpoint = absoluteEncoder.getPosition();

      BaseMotor.restoreFactoryDefaults();
      relativeEncoder = BaseMotor.getEncoder();
      RotaryPID.setReference(setpoint, CANSparkMax.ControlType.kPosition);

      RotaryPID.setP(RotaryConstants.kp);
      RotaryPID.setI(RotaryConstants.ki);
      RotaryPID.setD(RotaryConstants.kd);
      RotaryPID.setFF(RotaryConstants.kFF);

      RotaryPID.setOutputRange(RotaryConstants.kMinOutput, RotaryConstants.kMaxOutput);

      BaseMotor.setOpenLoopRampRate(RotaryConstants.kRampRate);

      relativeEncoder.setPositionConversionFactor(RotaryConstants.ConversionFactor);

      BaseMotor.setSmartCurrentLimit(RotaryConstants.kCurrentLimit);

      relativeEncoder.setPosition(absoluteEncoder.getPosition() * RotaryConstants.kGearing);

      //soft limits change later
      BaseMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, RotaryConstants.kEnableForwardLimit);
      BaseMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, RotaryConstants.kEnableReverseLimit);
      BaseMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)RotaryConstants.kForwardLimit);
      BaseMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)RotaryConstants.kReverseLimit);

      BaseMotor.setIdleMode(IdleMode.kBrake);

      BaseMotor.burnFlash();


      SmartDashboard.putNumber("Encoder Position", relativeEncoder.getPosition());
      SmartDashboard.putNumber("Encoder Velocity", relativeEncoder.getVelocity());


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