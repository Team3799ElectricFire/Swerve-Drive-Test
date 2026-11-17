package frc.robot.Utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule_SDS_Mk5n extends SwerveModule {
    private TalonFX SteerMotor, DriveMotor;
    private TalonFXConfiguration SteerConfig, DriveConfig;
    // private [?] SteerEncoder;
    // private [?] DriveEncoder;
    // private [?] DrivePID, SteerPID;
    @Logged
    private SwerveModuleState DesiredState = new SwerveModuleState(0.0, new Rotation2d());
    
    // Constants List
    private static final double maxModuleAttainableSpeed = Units.feetToMeters(22.5); // meters/second
    private static final double kDriveMotorCurrentStatorLimitAmps = 120;
    private static final double kDriveMotorCurrentSupplyLimitAmps = 70;
    private static final double kSteeringMotorCurrentStatorLimitAmps = 60;
    private static final double SteeringPgain = 2.5;
    private static final double SteeringIgain = 0.001;
    private static final double SteeringDgain = 3;
    private static final double SteerMotorPositionFactor = (2 * Math.PI); // radians per rotation
    private static final double DriveMotorPositionFactor = Math.PI * Units.inchesToMeters(4) / 5.27; // meters per rotation of motor
    // private static final double DriveMotorVelocityFactor = DriveMotorPositionFactor/60.0; // meters per sec
    private static final double DrivingPgain = 0.25;
    private static final double DrivingIgain = 0;
    private static final double DrivingDgain = 0;
    // private static final double DrivingIzone = 0.25;
    private static final double DrivingKs = 0.12;
    private static final double DrivingKv = 2.02708;


    public SwerveModule_SDS_Mk5n(int driveMotorID, int steerMotorID, int steerEncoderID, double steerOffset) {
        // Motors
        SteerMotor = new TalonFX(steerMotorID);
        DriveMotor = new TalonFX(driveMotorID);

        // Motor Configuration
        SteerConfig = new TalonFXConfiguration();
        SteerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO check direction
        SteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        SteerConfig.CurrentLimits.StatorCurrentLimit = kSteeringMotorCurrentStatorLimitAmps; // TODO check lim
        SteerConfig.Feedback.FeedbackRemoteSensorID = steerEncoderID;
        SteerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // TODO try fused encoder
        SteerConfig.Slot0.kP = SteeringPgain;
        SteerConfig.Slot0.kI = SteeringIgain;
        SteerConfig.Slot0.kD = SteeringDgain;
        SteerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        SteerConfig.Feedback.RotorToSensorRatio = 287.0 / 11.0;
        SteerConfig.Feedback.SensorToMechanismRatio = SteerMotorPositionFactor;
        SteerConfig.Feedback.FeedbackRotorOffset = steerOffset;
        SteerMotor.getConfigurator().apply(SteerConfig);

        DriveConfig = new TalonFXConfiguration();
        DriveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO check direction
        DriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        DriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        DriveConfig.CurrentLimits.StatorCurrentLimit = kDriveMotorCurrentStatorLimitAmps;
        DriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        DriveConfig.CurrentLimits.SupplyCurrentLimit = kDriveMotorCurrentSupplyLimitAmps;
        DriveConfig.Feedback.SensorToMechanismRatio = DriveMotorPositionFactor;
        DriveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        DriveConfig.Slot0.kP = DrivingPgain;
        DriveConfig.Slot0.kI = DrivingIgain;
        DriveConfig.Slot0.kD = DrivingDgain;
        DriveConfig.Slot0.kS = DrivingKs;
        DriveConfig.Slot0.kV = DrivingKv;
        DriveMotor.getConfigurator().apply(DriveConfig);
    }

    @Logged
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                DriveMotor.getVelocity().getValueAsDouble(),
                new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
    }

    public SwerveModuleState getDesiredPosition() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                DriveMotor.getPosition().getValueAsDouble(),
                new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));
    }

    @Logged
    public double getHeading(){
        return SteerMotor.getPosition().getValueAsDouble();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean restrictRotation) {
        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        desiredState.optimize(new Rotation2d(SteerMotor.getPosition().getValueAsDouble()));

        // Always set the driving motor's speed
        DriveMotor.setControl(new VelocityVoltage(desiredState.speedMetersPerSecond));

        // But only set the steering motor's position if the driving motor is moving
        if ((Math.abs(desiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) || !restrictRotation) {
            SteerMotor.setControl(new PositionVoltage(desiredState.angle.getRadians()));
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
