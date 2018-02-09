package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.ShifterSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Shifts down to low gear.
 */
public class ShiftDown extends InstantCommand {
	
	public ShiftDown() {
		requires(Robot.shifterSub);
	}
	
    protected void execute() {
    		ShifterSub.shiftLow();
    }
}
