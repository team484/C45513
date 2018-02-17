package org.usfirst.frc.team484.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.SpeedController;

public class TogglableSpeedController<T extends SpeedController> implements SpeedController {
	protected boolean enabled = true;
	protected final T controller;
	
	public TogglableSpeedController(T controller) {
		this.controller = controller;
	}
	
	public synchronized T getController() {
		return controller;
	}
	
	public boolean isEnabled() {
		return enabled;
	}
	
	public synchronized void enable() {
		enabled = true;
		
		if (controller instanceof BaseMotorController)
			((BaseMotorController) controller).setNeutralMode(NeutralMode.Brake);
	}
	
	@Override
	public synchronized void pidWrite(double output) {
		if (enabled) controller.pidWrite(output);
	}

	@Override
	public synchronized void set(double speed) {
		if (enabled) controller.set(speed);
	}

	@Override
	public double get() {
		return controller.get();
	}

	@Override
	public synchronized void setInverted(boolean isInverted) {
		controller.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		return controller.getInverted();
	}

	@Override
	public synchronized void disable() {
		controller.set(0);
		controller.disable();
		enabled = false;
		
		if (controller instanceof BaseMotorController)
			((BaseMotorController) controller).setNeutralMode(NeutralMode.Coast);
	}

	@Override
	public synchronized void stopMotor() {
		controller.stopMotor();
	}

}
