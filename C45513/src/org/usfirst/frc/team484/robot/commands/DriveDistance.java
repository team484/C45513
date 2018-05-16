package org.usfirst.frc.team484.robot.commands;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drives the robot a given distance.
 */
public class DriveDistance extends Command {
	private static PIDController pid = new PIDController(RobotSettings.DRIVE_DISTANCE_KP, RobotSettings.DRIVE_DISTANCE_KI, RobotSettings.DRIVE_DISTANCE_KD,
			new PIDSource() {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {}

		@Override
		public double pidGet() {
			return RobotIO.getFusedEncoderDistance();
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}, new PIDOutput() {

		@Override
		public void pidWrite(double output) {
			if (output > 0.2) {
				DriveSub.linearDrive(output, RobotSettings.DRIVE_PID_DRIFT_OFFSET * Math.signum(output));
			} else {
				DriveSub.linearDrive(output, 0);
			}
		}
	}, RobotSettings.DRIVE_PID_UPDATE_RATE);
	private double setpoint;
	public DriveDistance(double distance) {
		requires(Robot.driveSub);
		setpoint = distance;
	}

	protected void initialize() {
		if (RobotIO.leftEncoder != null) {
			RobotIO.leftEncoder.reset();
		}
		if (RobotIO.rightEncoder != null) {
			RobotIO.rightEncoder.reset();
		}
		if (pid == null) return;
		pid.setAbsoluteTolerance(RobotSettings.DRIVE_PID_TOLERANCE);
		pid.setSetpoint(setpoint);
		pid.enable();
	}

	@Override
	public void execute() {
	}
	
	protected boolean isFinished() {
		if (pid == null) return false;
		return pid.onTarget() &&
				Math.abs(RobotIO.getFusedEncoderRate()) < RobotSettings.DRIVE_PID_RATE_TOLERANCE;
	}

	protected void end() {
		if (pid != null) {
			pid.disable();
		}
		DriveSub.doNothing();
	}
}
