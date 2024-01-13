package frc.robot.commands.Testbed;

import frc.robot.subsystems.Testbed;

import edu.wpi.first.wpilibj2.command.Command;


public class Piston extends Command {
    Testbed testbed;
    public Piston() {
        testbed = new Testbed();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // testbed.movepiston();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}