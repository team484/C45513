package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CloseGrabber extends Command {

    public CloseGrabber() {
    	requires(Robot.grabberSub);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		GrabberSub.closeGrabber();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }
}
