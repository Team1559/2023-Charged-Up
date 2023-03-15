package frc.robot.commands;

import static frc.robot.Constants.Grabber.SERVO_RANGE;
import static frc.robot.Constants.Grabber.TELEOP_ANGULAR_VELOCITY_PER_CYCLE;

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
        int dir = controller.getDpad();
        double newAngle = wrist.getAngle();
        if (dir == 90) {
            newAngle += TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        } else if (dir == 270) {
            newAngle -= TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        } else if (dir == 0) {
            newAngle = 0;
        } else {
            return;
        }
        // double angle = wrist.getAngle(); // remove
        if (newAngle > SERVO_RANGE / 2 - 2) {
            newAngle = SERVO_RANGE / 2 - 2;
        } else if (newAngle < -SERVO_RANGE / 2 + 2) {
            newAngle = -SERVO_RANGE / 2 + 2;
        }
        wrist.setAngle(newAngle);
    }
}
