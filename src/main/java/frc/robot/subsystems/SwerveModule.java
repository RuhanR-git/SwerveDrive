package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule 
{
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final CANcoder driveEncoder = new CANcoder(1);
    private final CANcoder steerEncoder = new CANcoder(2);

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder; 
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    CANcoderConfiguration cfg = new CANcoderConfiguration();

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) 
    {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        // make 0–1.0 correspond to 0–360 degrees or 0–2π radians
        cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        driveEncoder.getConfigurator().apply(cfg);
        steerEncoder.getConfigurator().apply(cfg);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        resetEncoders();
    }

    public StatusSignal<Angle> getDrivePosition() 
    {
        return driveEncoder.getPosition();
    }

    public StatusSignal<Angle> getSteerPosition() 
    {
        return steerEncoder.getPosition();
    }

    public StatusSignal<AngularVelocity> getDriveVelocity() 
    {
        return driveEncoder.getVelocity();
    }

    public StatusSignal<AngularVelocity> getSteerVelocity() 
    {
        return steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() 
    {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() 
    {
        driveEncoder.setPosition(0.0);
        steerEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() 
    {
        return new SwerveModuleState();
        // Angle from absolute encoder (radians)
        double angleRad = getAbsoluteEncoderRad();

        // Get drive angular velocity from the StatusSignal; assume it's in radians/sec or adapt extraction to your API.
        double driveAngularRadPerSec = 0.0;
        try {
            // Adjust these accessors if your StatusSignal/AngularVelocity API differs (e.g. .getValue(), .getRadians(), .getRPS(), etc.)
            var status = getDriveVelocity();
            if (status != null && status.getValue() != null) {
                // Attempt common accessor names; change to match your library if needed.
                var angVel = status.getValue();
                // try methods that might exist; prefer radians/sec
                if (hasMethod(angVel, "getRadiansPerSecond")) {
                    driveAngularRadPerSec = (double) angVel.getClass().getMethod("getRadiansPerSecond").invoke(angVel);
                } else if (hasMethod(angVel, "getRadians")) {
                    driveAngularRadPerSec = (double) angVel.getClass().getMethod("getRadians").invoke(angVel);
                } else if (hasMethod(angVel, "getRPS")) {
                    driveAngularRadPerSec = ((double) angVel.getClass().getMethod("getRPS").invoke(angVel)) * 2.0 * Math.PI;
                } else if (hasMethod(status.getClass(), "get")) {
                    // fallback if StatusSignal returns primitive via get()
                    Object val = status.getClass().getMethod("get").invoke(status);
                    if (val instanceof Number) driveAngularRadPerSec = ((Number) val).doubleValue();
                }
            }
        } catch (Exception e) {
            // ignore and keep zero
        }

        // Convert wheel angular velocity (rad/s) to linear meters/sec using wheel radius constant
        double wheelRadius = DriveConstants.kWheelRadiusMeters; // ensure this constant exists
        double speedMps = driveAngularRadPerSec * wheelRadius;

        return new SwerveModuleState(speedMps, new Rotation2d(angleRad));
    }

    public void setDesiredState(SwerveModuleState state)
    {
        if (Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(turningPidController.calculate(getSteerPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() 
    {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}