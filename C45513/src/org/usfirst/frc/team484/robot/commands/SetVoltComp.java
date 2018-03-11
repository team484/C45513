package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.RobotIO;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetVoltComp extends InstantCommand {
	boolean val;
    public SetVoltComp(boolean val) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.val = val;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotIO.setVoltageComp(val);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }
}
