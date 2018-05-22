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
public class DriveUntilCube extends Command {
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
			DriveSub.pidOut1 = output;
			DriveSub.doublePIDDrive();
		}
	}, RobotSettings.DRIVE_PID_UPDATE_RATE);
	
	private static PIDController pid2 = new PIDController(RobotSettings.MAINTAIN_ANGLE_KP,
														RobotSettings.MAINTAIN_ANGLE_KI,
														RobotSettings.MAINTAIN_ANGLE_KD,
														new PIDSource() {
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) { }
		
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
			DriveSub.pidOut2 = -output;
			DriveSub.doublePIDDrive();
		}
	}, RobotSettings.ROTATE_PID_UPDATE_RATE);
	
	private double setpoint;
    public DriveUntilCube(double setpoint) {
        requires(Robot.driveSub);
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
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
    	
    	
    	double[] ypr = new double[3];
		if (RobotIO.imu != null) {
			RobotIO.imu.getYawPitchRoll(ypr);
		}
		if (pid2 == null) return;
		pid2.setSetpoint(ypr[0]);
		pid2.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
		pid2.enable();
		pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return (pid.onTarget() &&
				Math.abs(RobotIO.getFusedEncoderRate()) < RobotSettings.DRIVE_PID_RATE_TOLERANCE) || isAtCube();
    }

    // Called once after isFinished returns true
    protected void end() {
    	pid.disable();
    	pid2.disable();
    	DriveSub.doNothing();
    }
    
    private boolean isAtCube() {
    	double ir = RobotIO.irSensor.getAverageVoltage();
    	System.out.println(ir);
    	return ir > 1.25 && ir < 2.2;
    }

}
