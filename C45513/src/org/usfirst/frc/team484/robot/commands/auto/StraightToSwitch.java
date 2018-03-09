package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.commands.DriveStraight;
import org.usfirst.frc.team484.robot.commands.ElevateToHeight;
import org.usfirst.frc.team484.robot.commands.OpenGrabber;
import org.usfirst.frc.team484.robot.commands.RotateGrabberDown;
import org.usfirst.frc.team484.robot.commands.RotateGrabberUp;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;

/**
 * Drives forward and drops a cube into the switch.
 */
public class StraightToSwitch extends CommandGroup {

    public StraightToSwitch() {
    		addSequential(new ShiftUp(), 0.1);
		addParallel(new DriveStraight(109), 2.5); //Total distance - robot length - bumper thickness
		addParallel(new ElevateToHeight(RobotSettings.SWITCH_HEIGHT, 1), 1);
		addParallel(new RotateGrabberDown(0.6), 1.5);
		addSequential(new WaitForChildren());
		addSequential(new OpenGrabber(), 0.1);
		addSequential(new WaitCommand(1));
		addSequential(new RotateGrabberUp(1), 1);
    }
}
