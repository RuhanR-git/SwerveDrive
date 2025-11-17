package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

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

    @SuppressWarnings("deprecation")
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) 
    {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder.getConfigurator().apply(new CANcoderConfiguration() {{}});

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
        steerEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState() {
        // Get drive velocity in meters per second
        // StatusSignal<AngularVelocity> --> double linear velocity
        StatusSignal<AngularVelocity> velocitySignal = driveEncoder.getVelocity(); // we need to convert rotations/s to
                                                                                   // m/s by using gear ratio and wheel
                                                                                   // dimensions ~ maddox

        double velocity = velocitySignal.getValueAsDouble(); // rotation/s * circum * gear Ratio

        velocity *= ModuleConstants.kWheelCircumferenceMeters; // rotations/s * circum = linear m/s

        Angle turningPosition = steerEncoder.getPosition().getValue(); // radians

        return new SwerveModuleState(velocity, new Rotation2d(turningPosition));

        // // Angle from absolute encoder (radians)
        // // Breaks a lot :(
        // // double angleRad = (double) getPosition().getValue().toRadians();
        // // double angleRad = (double) getAbsoluteEncoderRad();
 // // Commented just in case of error
        // double angleRad = getAbsoluteEncoderRad();

        //        
        // // Get drive angular velocity from the StatusSignal; assume it's in radians/sec or adapt extraction to your API.
        // le driveAngularRadPerSec = 0.0;

        //        // {
        // // Adjust these accessors if your StatusSignal/AngularVelocity API differs (e.g. .getValue(), .getRadians(), .getRPS(), etc.)
        // status = getDriveVelocity();
        // status != null && status.getValue() != null) {
        // // Attempt common accessor names; change to match your library if needed.
        // var angVel = status.getValue();
        // // try methods that might exist; prefer radians/sec
        // hasMethod(angVel, "getRadiansPer
        //Second")) 
        // {
        //     driveAngularRadPerSec = (double) angVel.getClass().getMethod("getRadiansPerSecond").invoke(angVel);
        // } 
        //  if (hasMethod(angVel, "getRadia
        //ns")) 
        // {
        //     driveAngularRadPerSec = (double) angVel.getClass().getMethod("getRadians").invoke(angVel);
        // } 
        //  if (hasMethod(angVel, "getRPS"))
        // 
        // {
        //     driveAngularRadPerSec = ((double) angVel.getClass().getMethod("getRPS").invoke(angVel)) * 2.0 * Math.PI;
        // } 
        //  if (hasMethod(angVel, "get")) 
        //         // // fallback if StatusSignal returns primitive via get()

        //        //     Object val = status.getClass().getMethod("get").invoke(status);
        //         if (val instanceof Number number) driveAngularRadPerSec = number.doubleValue();
        //         }
        // }
        // } catch (Exception e) {
        //     // ignore and keep zero
        // }

        //

        //        // // Convert wheel angular velocity (rad/s) to linear meters/sec using wheel radius constant
        // double wheelRadius = DriveConstants.kWheelRadiusMeters; // ensure this constant exists
        // double speedMps = driveAngularRadPerSec * wheelRadius;
        // return new SwerveModuleState(speedMps, new Rotation2d(angleRad));
        
    }

    private boolean hasMethod(AngularVelocity angVel, String string) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasMethod'");
    }

    @SuppressWarnings("deprecation")
    public void setDesiredState(SwerveModuleState state)
    {
        if (Math.abs    (state.speedMetersPerSecond) < 0.001)
        {
            stop();
            return;
        }

        // double steerPos = getSteerPosition().getValueAsDouble();
        // Commented just in case of error
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        steerMotor.set(turningPidController.calculate(getSteerPosition().getValueAsDouble(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() 
    {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
    }
}