/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

	/**
	 * @param joystick the gamepad that the axis is on
	 * @param triggerPercent the amount the trigger must be pressed
	 * @param axis the axis on the joystick this  button reads
	 */
	public JoyTriggerButton(GenericHID joystick, double triggerPercent, int axis) {
		//pull the joystick axis, and see if larger than triggerPercent
		super(()-> (triggerPercent < joystick.getRawAxis(axis)));				
	}
	
	/**
	 * @param joystick the xboxcontroller that the axis is on
	 * @param triggerPercent the amount the trigger must be pressed
	 * @param axis the axis, in enum form, on the joystick this button reads
	 */
	public JoyTriggerButton(XboxController joystick, double triggerPercent, Axis axis) {
		this(joystick, triggerPercent, axis.value);
	}


}