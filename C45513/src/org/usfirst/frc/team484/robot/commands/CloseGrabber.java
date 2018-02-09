package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Closes the grabber arms.
 */
public class CloseGrabber extends InstantCommand {

    public CloseGrabber() {
    		requires(Robot.grabberSub);
    }

    protected void execute() {
    		GrabberSub.closeGrabber();
    }

}
