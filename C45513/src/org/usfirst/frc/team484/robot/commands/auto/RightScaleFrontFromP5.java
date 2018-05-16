package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.DriveUsingTrajectory;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateDownToHeight;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives to the front of the right scale from position 5.
 * The robot then deposits the cube on the scale and lowers the elevator.
 */
public class RightScaleFrontFromP5 extends CommandGroup {

    public RightScaleFrontFromP5() {	
    	addSequential(new ShiftUp(), 0.1);
    	addParallel(new DriveUsingTrajectory("RightScaleFrontFromP5"));
    	addSequential(new WaitCommand(2));
		addSequential(new PIDElevateUpToHeight(1.0), 2.5);
		addSequential(new WaitForChildren());
		addSequential(new RotateGrabberDown(0.4), 0.4);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new RotateGrabberDown(0.5), 0.5);
		addSequential(new WaitCommand(0.3), 0.3);
		CommandGroup reverseAndTurn = new CommandGroup();
		reverseAndTurn.addSequential(new DriveStraight(-50, 0.65), 3.0);
		reverseAndTurn.addParallel(new ShiftDown(), 0.1);
		reverseAndTurn.addSequential(new DriveAngle(160));
		reverseAndTurn.addSequential(new ShiftUp(), 0.1);
		addParallel(reverseAndTurn);
		addSequential(new WaitCommand(0.2),0.2);
		addParallel(new RotateGrabberUp(1), 1);
		addParallel(new PIDElevateDownToHeight(0), 3);
		addSequential(new WaitForChildren(), 3);
    }
}
