package frc.robot.subsystems.grabber;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class GrabberWrist extends SubsystemBase{
    
    private Servo wristServo;
    private int lastSet;
    
    public GrabberWrist(){
        wristServo=new Servo(WRIST_SERVO_PORT);
        lastSet=0;
    }
    public int getLastSet(){
        return lastSet;
    }
    public void setWrist(int angle){
        wristServo.setAngle(angle);
        lastSet=angle;
    }

}