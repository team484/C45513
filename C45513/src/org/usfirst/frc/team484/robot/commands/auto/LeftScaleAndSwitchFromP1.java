package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.CloseGrabber;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.RotateToCube;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives forward then turns right to drop the cube in the scale.
 */
public class LeftScaleAndSwitchFromP1 extends CommandGroup {

    public LeftScaleAndSwitchFromP1() {	
    		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(254.5), 4.5);
		addParallel(new ElevateToHeight(1.0, 1), 2.5);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-45), 1.0);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new WaitForChildren());
		addParallel(new DriveStraight(30),2);
		addSequential(new RotateGrabberDown(0.5), 1);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(0.5));
		addParallel(new DriveStraight(-20), 2);
		addParallel(new RotateGrabberUp(1), 1);
		addParallel(new CloseGrabber(), 0.1);
		addSequential(new WaitForChildren());
		addSequential(new ElevateToHeight(0, 1), 2.5);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-100), 1.5);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(58.0), 3);
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-55), 1);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new WaitCommand(0.3));
		addSequential(new RotateToCube(), 1);
		addParallel(new RotateGrabberDown(0.6), 1);
		addParallel(new OpenGrabber(), 0.1);
		addSequential(new DriveStraight(13), 1.5);
		addParallel(new CloseGrabber(), 0.1);
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1), 1);
		addParallel(new DriveStraight(-10), 1);
		addSequential(new WaitForChildren());
		addSequential(new DriveStraight(15), 1.2);
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new RotateGrabberUp(1), 1);
    }
}
