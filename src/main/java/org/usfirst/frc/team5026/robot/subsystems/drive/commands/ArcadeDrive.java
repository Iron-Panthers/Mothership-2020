/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5026.robot.subsystems.drive.commands;

import org.usfirst.frc.team5026.robot.Robot;
import org.usfirst.frc.team5026.robot.util.JoystickWrapper;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {

	private double leftPower, rightPower;
	private JoystickWrapper stick = Robot.oi.stick1;

	public ArcadeDrive() {
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		stick.update();
		rightPower = stick.findRightPower() + stick.skim(stick.findLeftPower());
		leftPower = stick.findLeftPower() + stick.skim(stick.findRightPower());
		double[] powers = { leftPower, rightPower };
		powers = scalePower(powers);
		// Uses ScalePower method to scale the power of both sides if either is over 1
		Robot.drive.set(powers[0], powers[1]);
	}

	/**
	 * Scales an array of powers
	 * 
	 * @param powers An array of powers, left power, then right power
	 * @return An array of scaled powers
	 */
	public double[] scalePower(double[] powers) {
		// Checks if the array needs to be scaled
		boolean isOverOne = false;
		for (double element : powers) {
			if (Math.abs(element) > 1) {
				isOverOne = true;
				break;
			}
		}
		if (!isOverOne) {
			return powers;
		}
		// Finds the highest value
		double greatestPower = powers[0];
		if (Math.abs(powers[1]) > greatestPower) {
			greatestPower = Math.abs(powers[1]);
		}
		// Scales all values
		for (int i = 0; i < powers.length; i++) {
			powers[i] /= greatestPower;
		}
		
		return powers;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
