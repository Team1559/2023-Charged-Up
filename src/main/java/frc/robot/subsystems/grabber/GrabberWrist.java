package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Grabber.SERVO_RANGE;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;
import static frc.robot.Constants.Wiring.WRIST_SERVO_PORT;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberWrist extends SubsystemBase {
    private Servo wristServo;

    public GrabberWrist() {
        wristServo = new Servo(WRIST_SERVO_PORT);
    }

    public void setAngle(double grabberAngle) {
        wristServo.setAngle(grabberAngle + ZERO_ANGLE);
    }

    public double getAngle() {
        return wristServo.getAngle() - ZERO_ANGLE;
    }

    public Command setWristAngleCommand(double angle) {
        return new InstantCommand(() -> setAngle(angle), this);
        // new WaitCommand(
        // Math.abs(angle - getAngle()) / MAX_ANGULAR_VELOCITY));
    }

    public void resetWrist() {
        wristServo.setAngle(ZERO_ANGLE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Grabber Angle", wristServo.get() * SERVO_RANGE - ZERO_ANGLE);
    }
}
