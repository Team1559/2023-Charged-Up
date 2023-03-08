package frc.robot.commands;

import static frc.robot.Constants.Grabber.SERVO_RANGE;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;
import frc.robot.subsystems.grabber.GrabberWrist;

public class TeleopWristAngleCommand extends CommandBase {
    private DTXboxController controller;
    private GrabberWrist     wrist;
    private final double     deadband = 0.05;

    public TeleopWristAngleCommand(GrabberWrist wrist, DTXboxController controller) {
        this.wrist = wrist;
        this.controller = controller;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        // If negative, rotate the wrist left by a set amount.
        // If positive, rotate the wrist right.
        double x = controller.getRightStickX();
        // double angle = wrist.getAngle(); // remove

        if (Math.abs(x) < deadband) {
            x = 0;
        }
        double newAngle = wrist.getAngle() + x;
        if (newAngle > SERVO_RANGE / 2 - 2) {
            newAngle = SERVO_RANGE / 2 - 2;
        } else if (newAngle < -SERVO_RANGE / 2 + 2) {
            newAngle = -SERVO_RANGE / 2 + 2;
        }
        wrist.setAngle(newAngle);
        System.out.println(wrist.getAngle());

        // remove
        // if (Math.abs(x) > deadband) {
        // if (x < 0) {
        // angle -= TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        // } else {
        // angle += TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        // }
        // if (angle > MAXIMUN_WRIST_ANGLE) {
        // angle = 90;
        // }
        // if (angle < -MINIMUM_WRIST_ANGLE) {
        // angle = -90;
        // }
        // wrist.setAngle(angle);
        // }
        // remove
    }
}
