package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class JoystickDrive extends Command {

    public JoystickDrive() {
        requires(Robot.driveSub);
    }

    protected void execute() {
    		if (RobotIO.driveStick == null) DriveSub.squaredDrive(0, 0);
    		DriveSub.squaredDrive(-RobotIO.driveStick.getY(), RobotIO.driveStick.getX());
    }

    protected boolean isFinished() { return false;  }

    protected void end() {
    		DriveSub.doNothing();
    }
}
