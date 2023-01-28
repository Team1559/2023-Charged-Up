package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Wiring.CLAW_SOLENOID_ID;
import static frc.robot.Constants.Wiring.PDH_ID;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberClaw extends SubsystemBase{
    //pneumatic solenoid
    private Solenoid clawSolenoid;
    private Solenoid clawPressure;
    public GrabberClaw(){
        clawSolenoid= new Solenoid(PDH_ID, PneumaticsModuleType.REVPH,CLAW_SOLENOID_ID);
    }
    public void openClaw(){
        clawSolenoid.set(true);
    }
    public void closeClaw(boolean pressure){
        clawPressure.set(pressure);
        
        clawSolenoid.set(false);
    }
    public boolean clawState(){
        return clawSolenoid.get();
    }
    public boolean pressureState(){
        return clawPressure.get();
    }
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Claw Open", clawState());
    }
}