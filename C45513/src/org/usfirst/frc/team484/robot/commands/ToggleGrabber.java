package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Toggles the grabber between open and close states
 */
public class ToggleGrabber extends InstantCommand {

    public ToggleGrabber() {
        requires(Robot.grabberSub);
    }

    protected void execute() {
    		GrabberSub.toggleGrabber();
    }
}
