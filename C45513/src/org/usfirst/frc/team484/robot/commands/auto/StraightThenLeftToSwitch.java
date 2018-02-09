package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveAngle;
import org.usfirst.frc.team484.robot.commands.DriveDistance;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives forward then turns left to drop the cube in the switch.
 */
public class StraightThenLeftToSwitch extends CommandGroup {

    public StraightThenLeftToSwitch() {
		addParallel(new DriveDistance(168.0 - 32.0/2.0 - 3.75)); //Total distance - 1/2 robot length - bumper thickness
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1));
		addParallel(new RotateGrabberDown(1));
		addSequential(new WaitForChildren());
		addSequential(new DriveAngle(-90));
		addSequential(new DriveDistance(40.75 - 32.0/2.0 - 3.75)); //Total distance - 1/2 robot length - bumper thickness
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addSequential(new DriveDistance(-20));
		addSequential(new RotateGrabberUp(1));
    }
}
