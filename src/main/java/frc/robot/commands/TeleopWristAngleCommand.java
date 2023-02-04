package frc.robot.commands;

// import static frc.robot.Constants.Grabber.MAXIMUN_WRIST_ANGLE;
// import static frc.robot.Constants.Grabber.MINIMUM_WRIST_ANGLE;
// import static frc.robot.Constants.Grabber.TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;
import frc.robot.subsystems.grabber.GrabberWrist;

public class TeleopWristAngleCommand extends CommandBase {
    private DTXboxController controller;
    private GrabberWrist     wrist;
    private final double     deadband = 0.05;

    public TeleopWristAngleCommand(GrabberWrist wrist,
            DTXboxController controller) {
        this.wrist = wrist;
        this.controller = controller;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        // If negative, rotate the wrist left by a set amount.
        // If positive, rotate the wrist right.
        double x = controller.getRightStickX();
        //double angle = wrist.getAngle();

        if(Math.abs(x)>deadband){
            if(x<(ZERO_ANGLE-deadband)||x>(ZERO_ANGLE+deadband)){
                wrist.resetWrist();
            }
        }
        else{
            wrist.setAngle(x);
        }

        // if (Math.abs(x) > deadband) {
        //     if (x < 0) {
        //         angle -= TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        //     } else {
        //         angle += TELEOP_ANGULAR_VELOCITY_PER_CYCLE;
        //     }
        //     if (angle > MAXIMUN_WRIST_ANGLE) {
        //         angle = 90;
        //     }
        //     if (angle < -MINIMUM_WRIST_ANGLE) {
        //         angle = -90;
        //     }
        //     wrist.setAngle(angle);
        // }
    }
}
