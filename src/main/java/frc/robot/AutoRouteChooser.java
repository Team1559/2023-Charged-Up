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
        positionChooser.addOption("Leave community (3)", routes::leave3);
        positionChooser.addOption("Score cone, leave (3) to GP 4", routes::scoreLeave3ToGamePiece4);
        positionChooser.addOption("Score cone, leave (1) to GP 1, Score Cube 1",
                routes::scoreLeave1ToGamePiece1ScoreCube);
        positionChooser.addOption("Score cone, leave (1) to GP 1", routes::scoreLeave1ToGamePiece1);
    }

    public Command getSelectedCommand() {
        return positionChooser.getSelected()
                              .get()
                              .beforeStarting(new PrintCommand("Auto started"));
    }
}
