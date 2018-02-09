package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberAngleSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Tells the grabber angle motor to remain stationary.
 */
public class GrabberAngleDoNothing extends Command {

    public GrabberAngleDoNothing() {
        requires(Robot.grabberAngleSub);
    }

    protected void execute() {
    		GrabberAngleSub.setRotateSpeed(0);
    }

    protected boolean isFinished() { return false; }

}
