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

/**
 * Rotates the robot to a given yaw delta.
 */
public class DriveAngle extends Command {
	ArrayList<Double> gyroAngles = new ArrayList<>();
	private PIDController pid;
	private double setpoint;
	public DriveAngle(double angle) {
		requires(Robot.driveSub);
		setpoint = angle;
		pid = new PIDController(RobotSettings.ROTATE_ANGLE_KP, RobotSettings.ROTATE_ANGLE_KI, RobotSettings.ROTATE_ANGLE_KD,
				new PIDSource() {

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {}

			@Override
			public double pidGet() {
				double[] ypr = new double[3];
				RobotIO.imu.getYawPitchRoll(ypr);
				gyroAngles.add(ypr[0]);
				while (gyroAngles.size() > RobotSettings.GYRO_SAMPLES_TO_AVERAGE) {
					gyroAngles.remove(0);
				}
				return ypr[0];
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		}, new PIDOutput() {

			@Override
			public void pidWrite(double output) {
				DriveSub.linearDrive(0, output);	
			}
		}, RobotSettings.ROTATE_PID_UPDATE_RATE);
	}

	protected void initialize() {
		double[] ypr = new double[3];
		RobotIO.imu.getYawPitchRoll(ypr);
		gyroAngles.clear();
		pid.setSetpoint(setpoint + ypr[0]);
		pid.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
		pid.enable();
	}

	protected boolean isFinished() {
		if (gyroAngles.size() == 0) return false;
		double averageRate = gyroAngles.get(gyroAngles.size() - 1) - gyroAngles.get(0);
		return pid.onTarget() &&
				Math.abs(averageRate) < RobotSettings.ROTATE_PID_RATE_TOLERANCE;
	}

	protected void end() {
		pid.disable();
		DriveSub.doNothing();
	}
}
