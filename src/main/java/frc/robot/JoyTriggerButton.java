/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * Converts a joystick trigger, which is a joystick 
 * axis value between 0.0 and 1.0 into a Trigger 
 * object, which can be used to bind a boolean state 
 * to a command.
 * {@code axis >= triggerPercent}
 */
public class JoyTriggerButton extends Trigger {
	// private XboxController stick;
	// private double percent;
	// private int axis;

	public JoyTriggerButton(GenericHID joystick, double triggerPercent, int axis) {
		super(()-> (triggerPercent < joystick.getRawAxis(axis)));
		
		// stick = joystick;
		// percent = triggerPercent;
		// this.axis = axis;
		
	}

	public JoyTriggerButton(XboxController joystick, double triggerPercent, Axis axis) {
		this(joystick, triggerPercent, axis.value);
	}

	/**
	 * @return {@code true} if axis >= givenPercent 
	 */
	// public boolean get() {
	// 	return stick.getRawAxis(axis) >= percent;
	// }
}