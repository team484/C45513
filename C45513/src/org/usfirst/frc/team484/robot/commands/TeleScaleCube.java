package org.usfirst.frc.team484.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 *
 */
public class TeleScaleCube extends CommandGroup {

	public TeleScaleCube() {
		addParallel(new ShiftUp(), 0.1);
		addParallel(new PIDElevateUpToHeight(1), 2);
		addSequential(new WaitForChildren());
		addSequential(new DriveStraight(40), 3);
		addSequential(new RotateGrabberDown(0.8), 0.3);
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(0.5),0.5);
		addSequential(new DriveStraight(-20), 2);
		addSequential(new PIDElevateDownToHeight(0), 2.5);
	}
}
