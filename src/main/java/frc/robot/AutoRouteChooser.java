package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.lib.NullCommand;

public class AutoRouteChooser {
    private final AutoRoutes                         routes;
    private final SendableChooser<Supplier<Command>> positionChooser;

    public AutoRouteChooser(AutoRoutes routes) {
        this.routes = routes;
        this.positionChooser = new SendableChooser<>();
        addOptions();
        SmartDashboard.putData("Auto Start Position", positionChooser);
    }

    private void addOptions() {
        positionChooser.setDefaultOption("Do Nothing (1)", NullCommand::new);
        positionChooser.addOption("Leave community (1)", routes::leave1);
        positionChooser.addOption("Score cone, stay", routes::scoreConeStay);
        positionChooser.addOption("Score cone, leave (1)", routes::scoreLeave1);
        positionChooser.addOption("Score cone, leave (3)", routes::scoreLeave3);
        // remove
        // positionChooser.addOption("Score, leave community, grab a piece (1)",
        // AutoRoute.SCORE_LEAVE_GRAB);
        // positionChooser.addOption("Score, leave community, grab a piece,
        // score (1)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE);
        // positionChooser.addOption(
        // "Score, leave community, grab a piece, score, leave the community
        // (1)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE_LEAVE);
        // positionChooser.addOption("Leave community (2)", AutoRoute.LEAVE_2);
        // positionChooser.addOption("Dock (2)", AutoRoute.DOCK);
        // positionChooser.addOption("Leave the community, dock (2)",
        // AutoRoute.LEAVE_DOCK);
        // positionChooser.addOption("Score and then Dock",
        // AutoRoute.SCORE_DOCK);
        // positionChooser.addOption("Score, leave community, dock (2)",
        // AutoRoute.SCORE_LEAVE_DOCK);
        // positionChooser.addOption("Score, leave the community, grab a piece,
        // dock (2)",
        // AutoRoute.SCORE_LEAVE_GRAB_DOCK);
        // remove
        positionChooser.addOption("Leave community (3)", routes::leave3);
        // remove
        // positionChooser.addOption("Score, leave community, grab a piece (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_3);
        // positionChooser.addOption("Score, leave community, grab a piece,
        // score (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE)_3;
        // positionChooser.addOption(
        // "Score, leave community, grab a piece, score, leave the community
        // (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE_LEAVE_3);
        // remove
    }

    public Command getSelectedCommand() {
        return positionChooser.getSelected()
                              .get()
                              .beforeStarting(new PrintCommand("Auto started"));
    }
}
