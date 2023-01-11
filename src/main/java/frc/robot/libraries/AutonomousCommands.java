// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.libraries;

/** Add your docs here. */
public enum AutonomousCommands {
    LOW_GOAL_OUT,
    HIGH_GOAL_D1,
    HIGH_GOAL_D2,
    DRIVE_PATH1;

    public String getSelectName() {
        return this.toString();
    }

    public int getSelectIx() {
        return this.ordinal();
    }
}
