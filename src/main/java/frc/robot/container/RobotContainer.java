// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.container;

import frc.robot.subsystem.*;
import lombok.Data;


public abstract @Data class RobotContainer {
	private SwerveDrive swerveDrive;
	private AlgaeCollector algaeCollector;
    private CoralCollectorIO coralCollector;


    public RobotContainer() {
		System.out.println("Constructed RobotContainer type: " + getClass());
	}
}
