package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.subsystems.ClimberSub;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * When running, the joystick's y axis is used to set climber speed.
 * This subsystem requires elevatorSub as well to avoid the joystick
 * interfering with elevator motion.
 */
public class ClimbWithJoystick extends Command {

    public ClimbWithJoystick() {
        requires(Robot.elevatorSub);
        requires(Robot.climberSub);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	ElevatorSub.setRate(0);
    	if (RobotIO.opStick == null) ClimberSub.climb(0);
    	ClimberSub.climb(RobotIO.opStick.getY());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	ClimberSub.climb(0);
    }
}
