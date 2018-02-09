package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Tells the elevator to hold position.
 */
public class ElevatorDoNothing extends Command {

    public ElevatorDoNothing() {
        requires(Robot.elevatorSub);
    }

    protected void execute() {
    		ElevatorSub.setRate(0);
    }

    protected boolean isFinished() { return false; }
}
