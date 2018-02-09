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
 * Robot drives forward then turns right to drop the cube in the scale.
 */
public class StraightThenRightToScale extends CommandGroup {

    public StraightThenRightToScale() {
		addParallel(new DriveDistance(307.25));
		addParallel(new ElevateToHeight(RobotSettings.LIFT_HEIGHT, 1));
		addParallel(new RotateGrabberDown(1));
		addSequential(new WaitForChildren());
		addSequential(new DriveAngle(90));
		addSequential(new DriveDistance(30.75 - 32.0/2.0 - 3.75)); //Total distance - 1/2 robot length - bumper thickness
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addSequential(new DriveDistance(-10));
		addSequential(new RotateGrabberUp(1));
		addSequential(new ElevateToHeight(0, 1));
    }
}
