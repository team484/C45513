package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.ClimberSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * This command sets the climber output to 0.
 */
public class ClimberDoNothing extends Command {

    public ClimberDoNothing() {
	     requires(Robot.climberSub);
    }

    protected void execute() {
    	ClimberSub.climb(0);
    }

    protected boolean isFinished() {
        return false;
    }
}
