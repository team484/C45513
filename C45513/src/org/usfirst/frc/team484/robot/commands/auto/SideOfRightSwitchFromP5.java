package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveUsingTrajectory;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateDownToHeight;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives forward then turns left to drop the cube in the switch.
 */
public class SideOfRightSwitchFromP5 extends CommandGroup {

    public SideOfRightSwitchFromP5() {
    	addSequential(new ShiftUp(), 0.1);
    	addParallel(new DriveUsingTrajectory("RightSwitchFromP5"));
    	addSequential(new WaitCommand(0.5));
		addParallel(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT),  1.5);
		addParallel(new RotateGrabberDown(0.6), 1);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addParallel(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT + 0.3), 1.5);
		addSequential(new WaitCommand(1));
		addParallel(new PIDElevateDownToHeight(RobotSettings.SWITCH_HEIGHT), 1.5);
    }
}
