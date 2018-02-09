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
 *
 */
public class DriveDistance extends Command {
	private PIDController pid;
	private double setpoint;
	public DriveDistance(double distance) {
		requires(Robot.driveSub);
		setpoint = distance;
		pid = new PIDController(RobotSettings.DRIVE_DISTANCE_KP, RobotSettings.DRIVE_DISTANCE_KI, RobotSettings.DRIVE_DISTANCE_KD,
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
				DriveSub.linearDrive(output, RobotSettings.DRIVE_PID_DRIFT_OFFSET);
			}
		}, RobotSettings.DRIVE_PID_UPDATE_RATE);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotIO.leftEncoder.reset();
		RobotIO.rightEncoder.reset();
		pid.setAbsoluteTolerance(RobotSettings.DRIVE_PID_TOLERANCE);
		pid.setSetpoint(setpoint);
		pid.enable();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return pid.onTarget() &&
				Math.abs(RobotIO.getFusedEncoderRate()) < RobotSettings.DRIVE_PID_RATE_TOLERANCE;
	}

	// Called once after isFinished returns true
	protected void end() {
		pid.disable();
		DriveSub.doNothing();
	}
}
