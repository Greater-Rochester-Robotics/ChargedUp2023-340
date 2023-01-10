/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.Trigger;



/**
 * Converts a trigger, locked to a joystick,
 * into a true/false button as if by
 * {@code axis >= triggerPercent}
 */
public class JoyTriggerButton extends Trigger {
	private XboxController stick;
	private double percent;
	private int axis;

	public JoyTriggerButton(XboxController joystick, double triggerPercent, int axis) {
		//super(() -> {stick.getRawAxis(axis) >= percent});
		super (

			new BooleanSupplier() {
				@Override
				public boolean getAsBoolean() {
					// TODO Auto-generated method stub
					return  stick.getRawAxis(axis) >= percent;
				}
			}
		);
		stick = joystick;
		percent = triggerPercent;
		this.axis = axis;
		
	}

	public JoyTriggerButton(XboxController joystick, double triggerPercent, Axis axis) {
		this(joystick, triggerPercent, axis.value);
	}

	/**
	 * @return {@code true} if axis >= givenPercent 
	 */
	public boolean get() {
		return stick.getRawAxis(axis) >= percent;
	}
}