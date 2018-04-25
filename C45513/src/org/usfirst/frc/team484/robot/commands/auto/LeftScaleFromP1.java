package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.DriveUsingTrajectory;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateDownToHeight;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives to the front of the left scale from position 1.
 * The robot then deposits the cube on the scale and lowers the elevator.
 */
public class LeftScaleFromP1 extends CommandGroup {

    public LeftScaleFromP1() {	
    	addSequential(new ShiftUp(), 0.1);
    	addParallel(new DriveUsingTrajectory("LeftScaleFromP1"));
    	addSequential(new WaitCommand(2));
		addSequential(new PIDElevateUpToHeight(1.0), 2.5);
		addSequential(new WaitForChildren());
		addSequential(new RotateGrabberDown(0.5), 0.5);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(0.5));
		addParallel(new DriveStraight(-20), 1);
		addParallel(new RotateGrabberUp(1), 1);
		addSequential(new WaitForChildren());
		addSequential(new PIDElevateDownToHeight(0), 2.5);
    }
}
