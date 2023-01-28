package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

    public class ArmWrist extends SubsystemBase{ //Note that this class assumes a 0 - 180
                
        private Servo wristServo;
        

    public ArmWrist(){
        wristServo = new Servo(Constants.Wiring.ARM_SERVO_PORTNUM);
    }
    public void ZeroWrist(){
        wristServo.setAngle(Constants.Arm.ZERO_ANGLE); 
    }
   
    public void setWristAt(double angle){
        wristServo.setAngle(calculateServoAngle(angle));
    }
    private double calculateServoAngle(double targetAngle){
        return targetAngle * (1 / Constants.Arm.ARM_WRIST_GEAR_RATIO) - Constants.Arm.ZERO_ANGLE;
    }
}
