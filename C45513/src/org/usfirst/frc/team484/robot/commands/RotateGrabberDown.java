package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.GrabberAngleSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Rotates the grabber arm all the way down.
 */
public class RotateGrabberDown extends Command {
	double speed;
    public RotateGrabberDown(double speed) {
       requires(Robot.grabberAngleSub);
       this.speed = Math.abs(speed);
    }

    protected void execute() {
    		GrabberAngleSub.setRotateSpeed(-speed);
    }

    protected boolean isFinished() {
        return GrabberAngleSub.isDown();
    }

    protected void end() {
    		GrabberAngleSub.setRotateSpeed(0);
    }
}
