package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Controls the elevator using the operator joystick.
 */
public class JoystickElevator extends Command {

    public JoystickElevator() {
        requires(Robot.elevatorSub);
    }

    protected void execute() {
    		if (RobotIO.opStick == null) ElevatorSub.setRate(0);
    		ElevatorSub.setRate(-RobotIO.opStick.getY());
    }

    protected boolean isFinished() { return false; }

    protected void end() {
    		ElevatorSub.doNothing();
    }
}
