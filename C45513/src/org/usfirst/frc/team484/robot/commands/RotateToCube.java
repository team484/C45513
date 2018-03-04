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
 * Uses computer vision code that runs during auto to adjust the robot heading
 * to point toward the closest cube.
 */
public class RotateToCube extends Command {
	private static PIDController pid = new PIDController(
			RobotSettings.ROTATE_ANGLE_KP,
			RobotSettings.ROTATE_ANGLE_KI,
			RobotSettings.ROTATE_ANGLE_KD,
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
	public RotateToCube() {
		requires(Robot.driveSub);
		pid.setOutputRange(-1.0, 1.0);
	}

	private long initTime;
	protected void initialize() {
		initTime = System.currentTimeMillis();
	}

	private double setpoint;
	protected void execute() {
		if (pid == null) return;
		if (pid.isEnabled()) return;
		synchronized (Robot.imgLock) {
			if (initTime > Robot.lastVisionUpdate) return;
			double[] ypr = new double[3];
			if (RobotIO.imu != null) {
				RobotIO.imu.getYawPitchRoll(ypr);
			}
			setpoint = Robot.visionCubeAngle;
			pid.setSetpoint(setpoint + ypr[0]);
			pid.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
			pid.enable();
		}
	}

	protected boolean isFinished() {
		if (pid == null) return true;
		if (!pid.isEnabled()) return false;
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
