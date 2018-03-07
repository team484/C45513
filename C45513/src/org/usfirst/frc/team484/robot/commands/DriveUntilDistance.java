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
 * Useful for quickly getting to a position and moving on to the next command where
 * accuracy is not needed or having the robot drift past the setpoint is preferred.
 */
public class DriveUntilDistance extends Command {
	double distance, speed;
	
	private PIDController pid = new PIDController(RobotSettings.MAINTAIN_ANGLE_KP,
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
			DriveSub.linearDrive(speed * Math.signum(distance), output);
		}
	}, RobotSettings.ROTATE_PID_UPDATE_RATE);
	
	/**
	 * This command drives until a set distance is reached and does so at a given speed.
	 * @param distance - the total distance to go.
	 * @param speed - [0.0 - 1.0] how fast you want to go there.
	 */
    public DriveUntilDistance(double distance, double speed) {
        requires(Robot.driveSub);
        this.distance = distance;
        this.speed = Math.abs(speed);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    		RobotIO.leftEncoder.reset();
    		RobotIO.rightEncoder.reset();
    		double[] ypr = new double[3];
			RobotIO.imu.getYawPitchRoll(ypr);
    		pid.setSetpoint(ypr[0]);
    		pid.enable();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return distance <= RobotIO.getFusedEncoderDistance();
    }

    // Called once after isFinished returns true
    protected void end() {
    	pid.disable();
    	DriveSub.doNothing();
    }
}
