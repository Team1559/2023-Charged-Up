package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Wiring.ARM_SERVO_PORTNUM;
import static frc.robot.Constants.Arm.ZERO_ANGLE;
import static frc.robot.Constants.Arm.ARM_WRIST_GEAR_RATIO;


    public class ArmWrist extends SubsystemBase{ //Note that this class assumes a 0 - 180
                
        private Servo wristServo;
        

    public ArmWrist(){
        wristServo = new Servo(ARM_SERVO_PORTNUM);
    }
    public void ZeroWrist(){
        wristServo.setAngle(ZERO_ANGLE); 
    }
    public void setWristAt(double angle){
        wristServo.setAngle((angle) + ZERO_ANGLE);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Angle", wristServo.get() * 180 - ZERO_ANGLE);
    }
}
