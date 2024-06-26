package frc.robot.library.bling.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.library.bling.BlingSubsystem;

/**
 * Default command for the Bling subsystem that sets the LEDs to the alliance color.
 */
public class Cmd_SubSys_Bling_DefaultSetToAllianceColor extends Command {
    private final BlingSubsystem subSysBling;

    /**
     * Constructs a new Cmd_SubSys_Bling_DefaultSetToAllianceColor command.
     *
     * @param subSysBling The Bling subsystem to control.
     */
    public Cmd_SubSys_Bling_DefaultSetToAllianceColor(BlingSubsystem subSysBling) {
        this.subSysBling = subSysBling;
        addRequirements(subSysBling);
    }

    @Override
    public void initialize() {
        subSysBling.setLedToAllianceColor();
    }

    @Override
    public void execute() {
        // The update is now called in the subsystem's periodic method
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to do as this is the default command
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true; // Set LED's when disabled
    }
}