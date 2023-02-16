package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase  {
    PWMSparkMax lmotor = new PWMSparkMax(0);
    PWMSparkMax rmotor = new PWMSparkMax(1);
    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0 ,1);
    //Change null and motors later
    public void CubeIn(){
        double speed = 0.1;
        lmotor.set(speed);
        rmotor.set(-speed);
    }
    public void ConeIn(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void CubeOut(){
        double speed = 0.1;
        lmotor.set(-speed);
        rmotor.set(speed);
    }
    public void ConeOut(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
}
