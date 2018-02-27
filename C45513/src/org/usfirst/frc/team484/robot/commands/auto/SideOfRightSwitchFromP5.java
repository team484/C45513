package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
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
 * Robot drives forward then turns left to drop the cube in the switch.
 */
public class SideOfRightSwitchFromP5 extends CommandGroup {

    public SideOfRightSwitchFromP5() {
    		addSequential(new ShiftUp(), 0.1);
		addParallel(new DriveStraight(168.0 - 32.0/2.0 - 3.75)); //Total distance - 1/2 robot length - bumper thickness
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1), 1.2);
		addParallel(new RotateGrabberDown(0.6));
		addSequential(new WaitForChildren());
		addSequential(new ShiftDown(), 0.1);
		addSequential(new DriveAngle(90));
		addSequential(new ShiftUp(), 0.1);
		addSequential(new DriveStraight(45)); //Total distance - 1/2 robot length - bumper thickness
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addSequential(new DriveStraight(-20));
		addSequential(new RotateGrabberUp(1), 1);
    }
}