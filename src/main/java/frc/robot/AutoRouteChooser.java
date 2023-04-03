package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.lib.NullCommand;

public class AutoRouteChooser {
    private final AutoRoutes                         routes;
    private final SendableChooser<Supplier<Command>> chooser;

    public AutoRouteChooser(AutoRoutes routes) {
        this.routes = routes;
        this.chooser = new SendableChooser<>();
        addOptions();
        SmartDashboard.putData("Auto Route", chooser);
    }

    private void addOptions() {
        chooser.addOption("(?) Sit and wait", NullCommand::new);
        chooser.setDefaultOption("(?) Score cone", routes::scoreConeStayCmd);
        chooser.addOption("(1) Leave", routes::leave1Cmd);
        chooser.addOption("(1) Score cone, leave", routes::scoreConeLeave1Cmd);
        chooser.addOption("(1) Score cone, leave, balance", routes::scoreConeLeave1BalanceCmd);
        chooser.addOption("(2) Score cone, balance", routes::scoreCone2BalanceCmd);
        chooser.addOption("(3) Leave", routes::leave3Cmd);
        chooser.addOption("(3) Score cone, leave", routes::scoreConeLeave3Cmd);
        chooser.addOption("(3) Score cone, leave, balance", routes::scoreConeLeave3BalanceCmd);
        chooser.addOption("(3) Score cone, leave, pickup cube",
                routes::scoreConeLeave3PickupCube4Cmd);
        chooser.addOption("(3) Score cone, leave, pickup cube, return",
                routes::scoreConeLeave3PickupCube4ReturnCmd);
    }

    public Command getSelectedCommand() {
        return chooser.getSelected()
                      .get()
                      .beforeStarting(new PrintCommand("Auto started"));
    }
}
