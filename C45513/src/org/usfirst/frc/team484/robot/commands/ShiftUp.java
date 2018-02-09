package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.subsystems.ShifterSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Shifts up to high gear.
 */
public class ShiftUp extends InstantCommand {

	public ShiftUp() {
		requires(Robot.shifterSub);
	}
	
	protected void execute() {
		ShifterSub.shiftLow();
	}
}
