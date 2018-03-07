package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.DriveUntilDistance;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Drives to the right switch from center (using 90 degree turns)
 */
public class RightSwitchFromP3V2 extends CommandGroup {
	private static final double DRIVE_SPEED = 0.6;
    public RightSwitchFromP3V2() {
        addSequential(new DriveUntilDistance(45.5, DRIVE_SPEED), 2);
        addSequential(new ShiftDown(), 0.1);
        addSequential(new DriveAngle(-45),3);
		addSequential(new ShiftUp(), 0.1);
        addSequential(new DriveUntilDistance(69.3, DRIVE_SPEED),3);
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1), 1);
		addParallel(new RotateGrabberDown(0.6));
		addSequential(new ShiftDown(), 0.1);
        addSequential(new DriveAngle(45),3);
		addSequential(new ShiftUp(), 0.1);
        addSequential(new DriveStraight(45.5),3);
        addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addSequential(new DriveStraight(-20));
		addSequential(new RotateGrabberUp(1), 1);
    }
}
