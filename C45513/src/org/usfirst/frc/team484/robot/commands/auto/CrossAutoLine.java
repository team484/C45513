package org.usfirst.frc.team484.robot.commands.auto;

import org.usfirst.frc.team484.robot.commands.DriveDistance;
import org.usfirst.frc.team484.robot.commands.ShiftUp;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Robot crosses the line and then stops in auto.
 */
public class CrossAutoLine extends CommandGroup {

    public CrossAutoLine() {
    		addSequential(new ShiftUp(), 0.1);
    		addSequential(new DriveDistance(90));
    }
}
