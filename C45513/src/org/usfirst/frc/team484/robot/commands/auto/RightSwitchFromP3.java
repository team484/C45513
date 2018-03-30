package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateDownToHeight;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Drives to the right switch from center (using 90 degree turns)
 */
public class RightSwitchFromP3 extends CommandGroup {

    public RightSwitchFromP3() {
        addSequential(new DriveStraight(33), 2);
        addSequential(new ShiftDown(), 0.1);
        addSequential(new DriveAngle(-90),1);
		addSequential(new ShiftUp(), 0.1);
        addSequential(new DriveStraight(53.5),3);
		addParallel(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT),  1.5);
		addParallel(new RotateGrabberDown(0.6), 1.5);
		addSequential(new ShiftDown(), 0.1);
        addSequential(new DriveAngle(90),1);
		addSequential(new ShiftUp(), 0.1);
        addSequential(new DriveStraight(80),3);
        addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(1));
		addParallel(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT + 0.3), 1.5);
		addSequential(new WaitCommand(1));
		addParallel(new PIDElevateDownToHeight(RobotSettings.SWITCH_HEIGHT), 1.5);

		//addSequential(new WaitCommand(1));
		//addSequential(new DriveStraight(-20), 1);
		//addSequential(new RotateGrabberUp(1), 1);
    }
}
