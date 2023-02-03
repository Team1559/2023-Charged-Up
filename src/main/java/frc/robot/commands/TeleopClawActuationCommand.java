package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.DTXboxController;
import frc.robot.subsystems.grabber.GrabberClaw;

public class TeleopClawActuationCommand extends CommandBase{
    private DTXboxController controller;
    private GrabberClaw claw;

    @Override
    public void execute(){
        boolean aButton = controller.getAButtonPressed();
        //close for cone ^
        boolean bButton = controller.getBButtonPressed();
        //close for cube ^
        boolean yButton = controller.getYButtonPressed();
        //open all the way ^
        if (aButton){
            claw.closeClaw(true);
        }
        else if (bButton){
            claw.closeClaw(false);
        }
        else if (yButton){
            claw.openClaw();
        }

    }
}
