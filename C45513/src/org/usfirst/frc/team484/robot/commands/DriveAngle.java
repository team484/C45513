package org.usfirst.frc.team484.robot.commands;

import java.util.ArrayList;

import org.usfirst.frc.team484.robot.Robot;
import org.usfirst.frc.team484.robot.RobotIO;
import org.usfirst.frc.team484.robot.RobotSettings;
import org.usfirst.frc.team484.robot.subsystems.DriveSub;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Rotates the robot to a given yaw delta.
 */
public class DriveAngle extends Command {
	private static PIDController pid = new PIDController(RobotSettings.ROTATE_ANGLE_KP, RobotSettings.ROTATE_ANGLE_KI, RobotSettings.ROTATE_ANGLE_KD,
			new PIDSource() {

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {}

		@Override
		public double pidGet() {
			if (RobotIO.imu == null) return 0;
			double[] ypr = new double[3];
			RobotIO.imu.getYawPitchRoll(ypr);
			return ypr[0];
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}, new PIDOutput() {

		@Override
		public void pidWrite(double output) {
			DriveSub.linearDrive(0, -output);	
		}
	}, RobotSettings.ROTATE_PID_UPDATE_RATE);
	private double setpoint;
	public DriveAngle(double angle) {
		requires(Robot.driveSub);
		setpoint = angle;
		pid.setOutputRange(-1.0, 1.0);
	}

	protected void initialize() {
		double[] ypr = new double[3];
		if (RobotIO.imu != null) {
			RobotIO.imu.getYawPitchRoll(ypr);
		}
		if (pid == null) return;
		pid.setSetpoint(setpoint + ypr[0]);
		pid.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
		pid.enable();
	}
	
	protected boolean isFinished() {
		if (pid == null) return true;
		if (setpoint < 0 && pid.getError() > 0) return true;
		if (setpoint > 0 && pid.getError() < 0) return true;
		return pid.onTarget();
	}

	protected void end() {
		if (pid != null) {
			pid.disable();
		}
		DriveSub.doNothing();
	}
}
