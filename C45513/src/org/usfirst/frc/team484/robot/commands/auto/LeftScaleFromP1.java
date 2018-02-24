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
 * Robot drives forward then turns right to drop the cube in the scale.
 */
public class LeftScaleFromP1 extends CommandGroup {

    public LeftScaleFromP1() {	
    		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(307.25));
		addParallel(new ElevateToHeight(1.0, 1));
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-90));
		addSequential(new ShiftUp(), 0.1);
		addSequential(new WaitForChildren());
		addParallel(new DriveStraight(25),2);
		addSequential(new RotateGrabberDown(0.5), 1);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(0.5));
		addParallel(new DriveStraight(-10));
		addParallel(new RotateGrabberUp(1), 1);
		addParallel(new CloseGrabber());
		addSequential(new WaitForChildren());
		addSequential(new ElevateToHeight(0, 1));
    }
}
