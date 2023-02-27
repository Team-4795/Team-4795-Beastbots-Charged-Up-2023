package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0 ,1);

    //Change null and motors later
    public EndEffector(){
        
    }
    public void intake(){
        solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void outake(){
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("pneumatics state", solenoid.get().toString());
    }
}