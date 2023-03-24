// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Converts a joystick trigger, which is a joystick axis value between 0.0 and 1.0 into a Trigger object, which can be used to bind a boolean state to a command.
 */
public class JoyTriggerButton extends Trigger {
	/**
     * Creates a new JoyTriggerButton.
	 * @param controller The controller that the axis is on.
	 * @param axis The axis on the controller this button reads.
	 * @param triggerPercent The amount the axis must be pressed.
	 */
	public JoyTriggerButton(GenericHID controller, int axis, double triggerPercent) {
		super(()-> (triggerPercent < controller.getRawAxis(axis)));				
	}

	/**
     * Creates a new JoyTriggerButton.
	 * @param controller The controller that the axis is on.
	 * @param axis The axis on the controller this button reads.
	 * @param triggerPercent The amount the axis must be pressed.
	 */
	public JoyTriggerButton(XboxController controller, Axis axis, double triggerPercent) {
		this(controller, axis.value, triggerPercent);
	}
}
