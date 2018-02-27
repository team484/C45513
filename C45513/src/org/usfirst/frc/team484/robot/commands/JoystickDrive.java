package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;
import org.usfirst.frc.team484.robot.subsystems.ShifterSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickDrive extends Command {

    public JoystickDrive() {
        requires(Robot.driveSub);
    }

    protected void execute() {
    		if (RobotIO.driveStick == null) {
    			DriveSub.linearDrive(0, 0);
    			return;
    		}
    		if (ShifterSub.isLow()) {
    			DriveSub.squaredDrive(-RobotIO.driveStick.getY(), RobotIO.driveStick.getX());
    		} else {
    			DriveSub.curvatureDrive(-RobotIO.driveStick.getY(), RobotIO.driveStick.getX());
    		}
    }

    protected boolean isFinished() { return false; }

    protected void end() {
    		DriveSub.doNothing();
    }
}
