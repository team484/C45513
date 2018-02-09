package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveDistance;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Drives forward and drops a cube into the switch.
 */
public class StraightToSwitch extends CommandGroup {

    public StraightToSwitch() {
		addParallel(new DriveDistance(140.0 - 32.0 - 6.5)); //Total distance - robot length - bumper thickness
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1));
		addParallel(new RotateGrabberDown(1));
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber());
		addSequential(new WaitCommand(1));
		addSequential(new DriveDistance(-20));
		addSequential(new RotateGrabberUp(1));
    }
}
