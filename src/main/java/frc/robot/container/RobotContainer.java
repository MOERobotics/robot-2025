// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.SwerveDrive;
import lombok.Data;

import javax.swing.text.Element;


public abstract @Data class RobotContainer {
	private SwerveDrive swerveDrive;
	private ElevatorSubsystem elevator;
	public RobotContainer() {
		System.out.println("Constructed RobotContainer type: " + getClass());
	}
}
