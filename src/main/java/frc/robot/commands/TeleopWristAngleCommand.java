package frc.robot.commands;

import static frc.robot.Constants.Grabber.SERVO_RANGE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;
import frc.robot.subsystems.grabber.GrabberWrist;

public class TeleopWristAngleCommand extends CommandBase {
    private DTXboxController controller;
    private GrabberWrist     wrist;

    public TeleopWristAngleCommand(GrabberWrist wrist, DTXboxController controller) {
        this.wrist = wrist;
        this.controller = controller;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        // If negative, rotate the wrist left by a set amount.
        // If positive, rotate the wrist right.
        double x = controller.getDpad();
        if (x == 90) {
            x = 1;
        } else if (x == 270) {
            x = -1;
        } else if (x == 0) {
            wrist.setAngle(0);
        } else {
            return;
        }
        // double angle = wrist.getAngle();
        double newAngle = wrist.getAngle() + x;
        if (newAngle > SERVO_RANGE / 2 - 2) {
            newAngle = SERVO_RANGE / 2 - 2;
        } else if (newAngle < -SERVO_RANGE / 2 + 2) {
            newAngle = -SERVO_RANGE / 2 + 2;
        }
        wrist.setAngle(newAngle);
        // System.out.println(wrist.getAngle());

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
    }
}
