package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Wiring.ARM_SERVO_PORTNUM;
import static frc.robot.Constants.Arm.ZERO_ANGLE;


    public class ArmWrist extends SubsystemBase{ //Note that this class assumes a 0 - 180 servo motor
    private Servo wristServo;
    public ArmWrist(){
        wristServo = new Servo(ARM_SERVO_PORTNUM);
    }
    public void ZeroWrist(){
        wristServo.setAngle(ZERO_ANGLE); 
    }
    public void setWristAt(double angle){ //IF SERVO IS NOT 0 - 180, change to values on a 0.0 to 1.0 basis
        wristServo.setAngle((angle) + ZERO_ANGLE);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Angle", wristServo.get() * 180 -  ZERO_ANGLE);
    }
}
