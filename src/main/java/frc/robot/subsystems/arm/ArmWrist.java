package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

    public class ArmWrist extends SubsystemBase{ //Note that this class assumes a 0 - 180
                
        private Servo wristServo;
        private int lastSet;

    public ArmWrist(){
        wristServo = new Servo(Constants.Wiring.ARM_SERVO_PORTNUM);
        lastSet = 0;
    }
    public void ZeroWrist(){
        wristServo.setAngle(0); //
    }
    public int getLastSet(){
        return lastSet;
    }
    public void setWristAt(int angle){
        wristServo.setAngle(angle);
        lastSet = angle;
    }
}