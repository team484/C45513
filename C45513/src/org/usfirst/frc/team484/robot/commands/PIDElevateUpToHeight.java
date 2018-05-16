package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.subsystems.ElevatorSub;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot a given distance.
 */
public class PIDElevateUpToHeight extends Command {
	private static PIDController pid = new PIDController(RobotSettings.ELEVATOR_UP_KP, RobotSettings.ELEVATOR_UP_KI, RobotSettings.ELEVATOR_UP_KD,
			new PIDSource() {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {}

		@Override
		public double pidGet() {
			return RobotIO.getElevatorHeight();
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}, new PIDOutput() {

		@Override
		public void pidWrite(double output) {
			ElevatorSub.setRate(output);
		}
	}, 0.01);
	private double setpoint;
	public PIDElevateUpToHeight(double height) {
		requires(Robot.elevatorSub);
		setpoint = height;
	}

	protected void initialize() {
		if (pid == null) return;
		pid.setSetpoint(setpoint);
		pid.setOutputRange(0.0, 1.0);
		pid.enable();
	}

	@Override
	public void execute() {
	}
	
	protected boolean isFinished() {
		return RobotIO.getElevatorHeight() > setpoint || ElevatorSub.isUp() || RobotIO.getElevatorHeight() > 0.9 && RobotIO.getElevatorRate() == 0;
	}

	protected void end() {
		if (pid != null) {
			pid.disable();
		}
		ElevatorSub.setRate(0);;
	}
}
