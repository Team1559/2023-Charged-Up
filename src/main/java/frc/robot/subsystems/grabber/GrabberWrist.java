package frc.robot.subsystems.grabber;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Wiring.WRIST_SERVO_PORT;
//import static frc.robot.Constants.Grabber.GRABBER_WRIST_GEAR_RATIO;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;
import static frc.robot.Constants.Grabber.SERVO_RANGE;

public class GrabberWrist extends SubsystemBase{
    
    private Servo wristServo;
     
    public GrabberWrist(){
        wristServo=new Servo(WRIST_SERVO_PORT);
    }
    public void setWrist(double grabberAngle){//Verify selected servo is 0-180
        wristServo.setAngle(grabberAngle+ZERO_ANGLE); //if not use wristServo.set() instead (0.0 to 1.0) 
    }
    public void resetWrist(){
        wristServo.setAngle(ZERO_ANGLE);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Grabber Angle", wristServo.get()*SERVO_RANGE-ZERO_ANGLE);
    }
    // private double calculateServoAngle(double targetAngle){
    //     return targetAngle+ZERO_ANGLE;
    // }
}