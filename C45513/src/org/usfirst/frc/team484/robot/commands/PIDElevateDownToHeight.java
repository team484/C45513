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
public class PIDElevateDownToHeight extends Command {
	private static PIDController pid = new PIDController(RobotSettings.ELEVATOR_DOWN_KP, RobotSettings.ELEVATOR_DOWN_KI, RobotSettings.ELEVATOR_DOWN_KD,
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
	public PIDElevateDownToHeight(double height) {
		requires(Robot.elevatorSub);
		setpoint = height;
	}

	protected void initialize() {
		if (pid == null) return;
		pid.setSetpoint(setpoint);
		pid.setOutputRange(-0.4, 0.0);
		pid.enable();
	}

	@Override
	public void execute() {
	}
	
	protected boolean isFinished() {
		System.out.println(setpoint + ", " + RobotIO.getElevatorHeight() + ", " + ElevatorSub.isDown());
		return RobotIO.getElevatorHeight() < setpoint || ElevatorSub.isDown();
	}

	protected void end() {
		if (pid != null) {
			pid.disable();
		}
		ElevatorSub.doNothing();
	}
}
