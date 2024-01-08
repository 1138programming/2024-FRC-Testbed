package frc.robot.commands.Base;

import frc.robot.subsystems.TestBed;

import edu.wpi.first.wpilibj2.command.Command;


public class Piston extends Command {
    TestBed testbed;
    public Piston() {
        testbed = new TestBed();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        testbed.movepiston();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}