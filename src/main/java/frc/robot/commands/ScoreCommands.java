package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.grabber.GrabberClaw;
import frc.robot.subsystems.grabber.GrabberWrist;

public class ScoreCommands {
    private ScoreCommands() {}

    public static Command zeroWrist(GrabberWrist wrist) {
        return wrist.setWristAngleCommand(0);
    }

    public static Command moveToTravel(Arm arm) {
        return arm.moveSequentially(Arm.Position.TRAVEL);
    }

    public static Command moveToConeHigh(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.UPPER_CONE);
    }

    public static Command moveToConeMid(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.MIDDLE_CONE);
    }

    public static Command moveToConeLow(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.LOWER_CONE);
    }

    public static Command moveToCubeHigh(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.UPPER_CUBE);
    }

    public static Command moveToCubeMid(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.MIDDLE_CUBE);
    }

    public static Command moveToCubeLow(Arm arm, GrabberWrist wrist) {
        return arm.moveSequentially(Arm.Position.LOWER_CUBE);
    }

    public static Command scoreConeHigh(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return ScoreCommands.moveToConeHigh(arm, wrist)
                            .alongWith(zeroWrist(wrist))
                            .andThen(new WaitCommand(.6))
                            .andThen(claw.openClawCommand());
    }

    public static Command scoreConeMid(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return ScoreCommands.moveToConeMid(arm, wrist)
                            .alongWith(zeroWrist(wrist))
                            .andThen(claw.openClawCommand());
    }

    public static Command scoreConeLow(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return ScoreCommands.moveToConeLow(arm, wrist)
                            .alongWith(zeroWrist(wrist))
                            .andThen(claw.openClawCommand());
    }

    public static Command scoreCubeHigh(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return arm.moveSequentially(Arm.Position.UPPER_CUBE)
                  .alongWith(zeroWrist(wrist))
                  .andThen(claw.openClawCommand());
    }

    public static Command scoreCubeMid(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return ScoreCommands.moveToCubeMid(arm, wrist)
                            .alongWith(zeroWrist(wrist))
                            .andThen(claw.openClawCommand());
    }

    public static Command scoreCubeLow(Arm arm, GrabberWrist wrist, GrabberClaw claw) {
        return ScoreCommands.moveToCubeLow(arm, wrist)
                            .alongWith(zeroWrist(wrist))
                            .andThen(claw.openClawCommand());
    }

    public static Command pickupConeCommand(Arm arm, GrabberClaw claw) {
        return arm.moveSequentially(Arm.Position.TRAVEL)
                  .andThen(claw.openClawCommand())
                  .andThen(arm.moveToPosition(Arm.Position.PICKUP_CONE))
                  .andThen(claw.closeClawCommand())
                  .andThen(arm.moveToPosition(Arm.Position.TRAVEL));
    }

    public static Command pickupCubeCommand(Arm arm, GrabberClaw claw) {
        return arm.moveSequentially(Arm.Position.TRAVEL)
                  .andThen(claw.openClawCommand())
                  .andThen(arm.moveToPosition(Arm.Position.PICKUP_CUBE)
                              .withTimeout(2))
                  .andThen(claw.closeClawCommand())
                  .andThen(arm.moveToPosition(Arm.Position.TRAVEL));
    }
}
