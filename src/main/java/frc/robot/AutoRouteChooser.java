package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRouteChooser {
    private final AutoRoutes               routes;
    private final SendableChooser<Command> positionChooser;

    public AutoRouteChooser(AutoRoutes routes) {
        this.routes = routes;
        this.positionChooser = new SendableChooser<>();
        addOptions();
        SmartDashboard.putData("Auto Start Position", positionChooser);
    }

    private void addOptions() {
        positionChooser.setDefaultOption("Do Nothing (1)", null);
        positionChooser.addOption("Leave community (1)", routes.LEAVE_1);
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
        positionChooser.addOption("Leave community (3)", routes.LEAVE_3);
        // positionChooser.addOption("Score, leave community, grab a piece (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_3);
        // positionChooser.addOption("Score, leave community, grab a piece,
        // score (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE)_3;
        // positionChooser.addOption(
        // "Score, leave community, grab a piece, score, leave the community
        // (3)",
        // AutoRoute.SCORE_LEAVE_GRAB_SCORE_LEAVE_3);
    }

    public Command getSelectedCommand() {
        return positionChooser.getSelected();
    }
}
