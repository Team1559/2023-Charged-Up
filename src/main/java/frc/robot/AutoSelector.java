package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSelector extends SubsystemBase {
    public enum StartPosition {
        POS_1,
        POS_2,
        POS_3;
    }

    public enum AutoRoute {
        NULL,
        LEAVE,
        DOCK,
        LEAVE_DOCK,
        SCORE_DOCK,
        SCORE_LEAVE_DOCK,
        SCORE_LEAVE_GRAB,
        SCORE_LEAVE_GRAB_DOCK,
        SCORE_LEAVE_GRAB_SCORE,
        SCORE_LEAVE_GRAB_SCORE_LEAVE;
    }

    private final SendableChooser<StartPosition> positionChooser;
    private final SendableChooser<AutoRoute>     routeChooser1;
    private final SendableChooser<AutoRoute>     routeChooser2;
    private final SendableChooser<AutoRoute>     routeChooser3;
    private StartPosition                        lastPos;

    public AutoSelector() {
        positionChooser = new SendableChooser<>();
        routeChooser1 = new SendableChooser<>();
        routeChooser2 = new SendableChooser<>();
        routeChooser3 = new SendableChooser<>();
        lastPos = null;
        addOptions();
        SmartDashboard.putData("Auto Start Position", positionChooser);
    }

    private void addOptions() {
        positionChooser.addOption("Position 1", StartPosition.POS_1);
        positionChooser.addOption("Position 2", StartPosition.POS_2);
        positionChooser.addOption("Position 3", StartPosition.POS_3);

        routeChooser1.setDefaultOption("Do Nothing", AutoRoute.NULL);
        routeChooser1.addOption("Leave community", AutoRoute.LEAVE);
        routeChooser1.addOption("Score, leave community, grab a piece",
                AutoRoute.SCORE_LEAVE_GRAB);
        routeChooser1.addOption("Score, leave community, grab a piece, score",
                AutoRoute.SCORE_LEAVE_GRAB_SCORE);
        routeChooser1.addOption(
                "Score, leave community, grab a piece, score, leave the community",
                AutoRoute.SCORE_LEAVE_GRAB_SCORE_LEAVE);

        routeChooser2.setDefaultOption("Do Nothing", AutoRoute.NULL);
        routeChooser2.addOption("Leave community", AutoRoute.LEAVE);
        routeChooser2.addOption("Dock", AutoRoute.DOCK);
        routeChooser2.addOption("Leave the community, dock",
                AutoRoute.LEAVE_DOCK);
        routeChooser2.addOption("Score and then Dock", AutoRoute.SCORE_DOCK);
        routeChooser2.addOption("Score, leave community, dock",
                AutoRoute.SCORE_LEAVE_DOCK);
        routeChooser2.addOption(
                "Score, leave the community, grab a piece, dock",
                AutoRoute.SCORE_LEAVE_GRAB_DOCK);

        routeChooser3.setDefaultOption("Do Nothing", AutoRoute.NULL);
        routeChooser3.addOption("Leave community", AutoRoute.LEAVE);
        routeChooser3.addOption("Score, leave community, grab a piece",
                AutoRoute.SCORE_LEAVE_GRAB);
        routeChooser3.addOption("Score, leave community, grab a piece, score",
                AutoRoute.SCORE_LEAVE_GRAB_SCORE);
        routeChooser3.addOption(
                "Score, leave community, grab a piece, score, leave the community",
                AutoRoute.SCORE_LEAVE_GRAB_SCORE_LEAVE);
    }

    public Command getCommand() {
        return null;
        // TODO: implement routine generation logic
    }

    @Override
    public void periodic() {
        StartPosition selection = positionChooser.getSelected();
        if (selection != lastPos) {
            switch (selection) {
                case POS_1:
                    SmartDashboard.putData("Route Selector", routeChooser1);
                    break;
                case POS_2:
                    SmartDashboard.putData("Route Selector", routeChooser2);
                    break;
                case POS_3:
                    SmartDashboard.putData("Route Selector", routeChooser3);
                    break;
            }
            lastPos = selection;
        }
    }
}
