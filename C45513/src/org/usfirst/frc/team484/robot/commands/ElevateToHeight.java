package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Raises the elevator to a given height.
 */
public class ElevateToHeight extends Command {
	boolean goUp;
	double height;
	double speed;

	public ElevateToHeight(double height, double speed) {
		requires(Robot.elevatorSub);
		this.height = height;
		this.speed = Math.abs(speed);
	}

	protected void initialize() {
		goUp = RobotIO.getElevatorHeight() < height;
	}

	protected void execute() {
		if (goUp) {
			ElevatorSub.setRate(speed);
		} else {
			ElevatorSub.setRate(-speed);
		}
	}

	protected boolean isFinished() {
		double actualHeight = 0.1;//RobotIO.elevatorEncoder.getDistance();
		if (goUp && ElevatorSub.isUp()) return true;
		if (!goUp && ElevatorSub.isDown()) return true;
		return goUp && height <= actualHeight ||
				!goUp && height >= actualHeight;
	}

	protected void end() {
		ElevatorSub.setRate(0);
	}

}
