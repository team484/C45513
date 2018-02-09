package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.subsystems.ShifterSub;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ShiftUp extends InstantCommand {

	protected void initialize() {
		ShifterSub.shiftLow();
	}
}
