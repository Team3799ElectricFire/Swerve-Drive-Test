// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule_SDS_Mk4i extends SwerveModule {
    private SparkFlex SteerMotor, DriveMotor;
    private SparkFlexConfig SteerConfig, DriveConfig;
    private AbsoluteEncoder SteerEncoder;
    private RelativeEncoder DriveEncoder;
    private SparkClosedLoopController DrivePID, SteerPID;
    @Logged
    private SwerveModuleState DesiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    // Constants List
    private static final double maxModuleAttainableSpeed = Units.feetToMeters(19.3); // meters/second
    private static final int kDriveMotorCurrentLimitAmps = 40;
    private static final int kSteeringMotorCurrentLimitAmps = 20;
    private static final double SteeringPgain = 2.5;
    private static final double SteeringIgain = 0.001;
    private static final double SteeringDgain = 3; // Max derivative gain is 3 (according to REV hardware client)
    private static final double SteerMotorPositionFactor = (2 * Math.PI); // radians per rotation
    private static final double DriveMotorPositionFactor = 0.0521375063; // meters
    private static final double DriveMotorVelocityFactor = DriveMotorPositionFactor/60.0; // meters per sec
    private static final double DrivingPgain = 0.25;
    private static final double DrivingIgain = 0;
    private static final double DrivingDgain = 0;
    private static final double DrivingIzone = 0.25;
    private static final double DrivingKs = 0.12;
    private static final double DrivingKv = 2.02708;


    public SwerveModule_SDS_Mk4i(int driveMotorID, int steerMotorID, double steerOffset) {
        // Motors
        SteerMotor = new SparkFlex(steerMotorID, MotorType.kBrushless);
        DriveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

        // Motor Configuration
        SteerConfig = new SparkFlexConfig();
        SteerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kSteeringMotorCurrentLimitAmps);
        SteerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .p(SteeringPgain)
            .i(SteeringIgain)
            .d(SteeringDgain)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI)
            .outputRange(-1, 1);
        SteerConfig.absoluteEncoder
            .zeroOffset(steerOffset)
            .positionConversionFactor(SteerMotorPositionFactor);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DriveConfig = new SparkFlexConfig();
        DriveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kDriveMotorCurrentLimitAmps);
        DriveConfig.encoder
            .positionConversionFactor(DriveMotorPositionFactor)
            .velocityConversionFactor(DriveMotorVelocityFactor);
        DriveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(DrivingPgain)
            .i(DrivingIgain)
            .d(DrivingDgain)
            .iZone(DrivingIzone)
            .outputRange(-1, 1);
        DriveConfig.closedLoop.feedForward
            .kS(DrivingKs)
            .kV(DrivingKv);
        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // encoders
        SteerEncoder = SteerMotor.getAbsoluteEncoder();
        DriveEncoder = DriveMotor.getEncoder();

        // PIDs
        SteerPID = SteerMotor.getClosedLoopController();
        DrivePID = DriveMotor.getClosedLoopController();
    }

    @Logged
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                DriveEncoder.getVelocity(),
                new Rotation2d(SteerEncoder.getPosition()));
    }

    public SwerveModuleState getDesiredPosition() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                DriveEncoder.getPosition(),
                new Rotation2d(SteerEncoder.getPosition()));
    }

    @Logged
    public double getHeading(){
        return SteerEncoder.getPosition();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean restrictRotation) {
        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        desiredState.optimize(new Rotation2d(SteerEncoder.getPosition()));

        // Always set the driving motor's speed
        DrivePID.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        // But only set the steering motor's position if the driving motor is moving
        if ((Math.abs(desiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) || !restrictRotation) {
            SteerPID.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
        }

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
       setDesiredState(desiredState, false);
    }

    public void stop() {
        DriveMotor.set(0.0);
        SteerMotor.set(0.0);
    }

    public static double getMaxAttainableSpeed(){
        return maxModuleAttainableSpeed;
    }
}
