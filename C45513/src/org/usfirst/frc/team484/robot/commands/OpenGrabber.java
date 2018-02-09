package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Opens the grabber arms.
 */
public class OpenGrabber extends InstantCommand {

    public OpenGrabber() {
        requires(Robot.grabberSub);
    }

    protected void execute() {
    		GrabberSub.openGrabber();
    }
}
