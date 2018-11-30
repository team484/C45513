package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Enables or disables drivetrain voltage compensation when run.
 */
public class SetVoltageComp extends InstantCommand {
	private boolean enabled;
	private double value;
	public SetVoltageComp(boolean enabled) {
		this.enabled = enabled;
		this.value = RobotSettings.VOLTAGE_TARGET;
	}
	
	public SetVoltageComp(boolean enabled, double value) {
		this.value = value;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		RobotIO.setVoltageComp(enabled, value);
	}
}
