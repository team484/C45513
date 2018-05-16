package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.CloseGrabber;
import org.usfirst.frc.team484.robot.commands.DriveUntilCube;
import org.usfirst.frc.team484.robot.commands.DriveUntilDistance;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.PIDElevateUpToHeight;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateToCube;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Robot drives to the front of the left scale from position 1.
 * The robot then deposits the cube and proceeds to grab a second
 * cube which is placed on the left switch.
 */
public class LeftScaleAndSwitchFromP1 extends CommandGroup {

    public LeftScaleAndSwitchFromP1() {	
    	addSequential( new LeftScaleFrontFromP1());
		addSequential(new WaitCommand(0.2));
		addSequential(new RotateToCube(), 0.7);
		addParallel(new RotateGrabberDown(0.6), 1);
		addParallel(new OpenGrabber(), 0.1);
		addSequential(new WaitForChildren());
		addSequential(new DriveUntilCube(25), 2.0);
		addSequential(new CloseGrabber(), 0.1);
		addSequential(new WaitCommand(0.3));
		addSequential(new PIDElevateUpToHeight(RobotSettings.SWITCH_HEIGHT), 1.0);
		addSequential(new DriveUntilDistance(10.0, 0.7), 0.8);
		addSequential(new OpenGrabber(), 0.1);
    }
}
