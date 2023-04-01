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
        positionChooser.setDefaultOption("Do nothing", NullCommand::new);
        positionChooser.addOption("Score cone", routes::scoreConeStayCmd);
        positionChooser.addOption("Leave (1)", routes::leave1Cmd);
        positionChooser.addOption("Score cone, leave (1)", routes::scoreConeLeave1Cmd);
        positionChooser.addOption("Score cone, leave (1), pickup cube (1)",
                routes::scoreConeLeave1PickupCube1Cmd);
        positionChooser.addOption("Score cone, leave (1), pickup cube (1), Return",
                routes::scoreConeLeave1PickupCube1ReturnCmd);
        positionChooser.addOption("Leave (3)", routes::leave3Cmd);
        positionChooser.addOption("Score cone, leave (3)", routes::scoreConeLeave3Cmd);
        positionChooser.addOption("Score cone, leave (3), pickup cube (4)",
                routes::scoreConeLeave3PickupCube4Cmd);
        positionChooser.addOption("Score cone, leave (3), pickup cube (4), score (3)",
                routes::scoreConeLeave3PickupCube4ScoreCmd);
        positionChooser.addOption("Auto balance", routes::balanceChargeStationCmd);
        positionChooser.addOption("Go over and balance", routes::goOverChargeStationThenBalanceCmd);
    }

    public Command getSelectedCommand() {
        return positionChooser.getSelected()
                              .get()
                              .beforeStarting(new PrintCommand("Auto started"));
    }
}
