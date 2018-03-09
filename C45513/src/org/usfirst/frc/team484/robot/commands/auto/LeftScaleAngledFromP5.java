package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.commands.CloseGrabber;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Goes to the left scale from the right side of the field
 */
public class LeftScaleAngledFromP5 extends CommandGroup {

	public LeftScaleAngledFromP5() {	
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(235.2), 4.5);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(90), 1.0);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(230.625), 4.5);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-90), 1.0);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(89.0-52.75), 3);
		addParallel(new ElevateToHeight(1.0, 1), 2.5);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-45), 1);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new WaitForChildren());
		addParallel(new DriveStraight(30),2);
		addSequential(new RotateGrabberDown(0.5), 1);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(0.5));
		addParallel(new DriveStraight(-10), 1);
		addParallel(new RotateGrabberUp(1), 1);
		addParallel(new CloseGrabber(), 0.1);
		addSequential(new WaitForChildren());
		addSequential(new ElevateToHeight(0, 1), 2.5);
	}
}
