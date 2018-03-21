package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftDown;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives forward then turns right to drop the cube in the switch.
 */
public class SideOfLeftSwitchFromP1 extends CommandGroup {

    public SideOfLeftSwitchFromP1() {
    		addSequential(new ShiftUp(), 0.1);
		addParallel(new DriveStraight(168.0 - 32.0/2.0 - 3.75), 2.5); //Total distance - 1/2 robot length - bumper thickness
		addParallel(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT),  1.5);
		addParallel(new RotateGrabberDown(0.6), 1);
		addSequential(new WaitForChildren());
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(-90), 1);
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(45),1.1); //Total distance - 1/2 robot length - bumper thickness
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addParallel(new DriveStraight(-20), 1);
		addSequential(new RotateGrabberUp(1), 1);
    }
}
