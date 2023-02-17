package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class EndEffector {
    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0 ,1);
    //Change null and motors later
    public EndEffector(){
        
    }
    public void Intake(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void Outake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void ConeStill(){
        solenoid.set(DoubleSolenoid.Value.kOff);
    }
}