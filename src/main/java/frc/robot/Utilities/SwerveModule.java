// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public abstract class SwerveModule {
    public abstract SwerveModuleState getState();
    public abstract SwerveModuleState getDesiredPosition();
    public abstract SwerveModulePosition getPosition();
    public abstract double getHeading();
    public abstract void setDesiredState(SwerveModuleState desiredState);
    public abstract void stop();
}
