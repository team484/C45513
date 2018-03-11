package org.usfirst.frc.team484.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class TeleScaleCube extends CommandGroup {

	public TeleScaleCube() {
		addParallel(new ElevateToHeight(1, 1), 2.5);
		addSequential(new RotateGrabberUp(1.0), 1);
		addSequential(new RotateGrabberDown(1.0), 0.1);
		addSequential(new WaitForChildren());
		addSequential(new DriveStraight(40), 3);
		addSequential(new RotateGrabberDown(0.8), 1);
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new DriveStraight(-20), 2);
		addSequential(new RotateGrabberUp(1.0), 1.0);
		addSequential(new ElevateToHeight(0, 1), 2.5);
	}
}
