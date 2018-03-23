package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.RobotIO;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables or disables drivetrain voltage compensation when run.
 */
public class SetVoltageComp extends InstantCommand {
	private boolean enabled;
	public SetVoltageComp(boolean enabled) {
		this.enabled = enabled;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotIO.setVoltageComp(enabled);
	}
}
