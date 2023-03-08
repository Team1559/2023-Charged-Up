package frc.lib;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// Explicitly does nothing when called (cleaner code)
public class NullCommand extends InstantCommand {
    public NullCommand() {
        super();
    }
}
