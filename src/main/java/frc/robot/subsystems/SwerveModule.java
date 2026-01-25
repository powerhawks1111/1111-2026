// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.*;
import com.revrobotics.*;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConst;

/**
 * This is the code to run a single swerve module <br><br>
 * It is called by the Drivetrain subsysem
 */
public class SwerveModule extends SubsystemBase {
        //components for the drive section of the module
        private SparkFlex m_driveMotor;
        private SparkFlexConfig m_driveMotorConfig;
        private RelativeEncoder m_driveEncoder;
        private MAXMotionConfig m_MaxMotionConfig;

        private SparkFlexConfig m_DRSConfig;

        //components for the turning section of the module. 
        private SparkMax m_turningMotor;
        private SparkMaxConfig m_turningMotorConfig;
        private AbsoluteEncoder m_turningEncoder;

        private SparkClosedLoopController m_driveController;
        private SparkClosedLoopController m_turnController;

        private SwerveModuleState m_state = new SwerveModuleState();
        private SwerveModulePosition m_position = new SwerveModulePosition();
        /**
         * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
         *
         * @param driveMotorChannel CAN ID for the drive motor.
         * @param turningMotorChannel CAN ID for the turning motor.
         */
        public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
            m_driveMotor = new SparkFlex(driveMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_driveMotorConfig = new SparkFlexConfig();
            m_driveEncoder = m_driveMotor.getEncoder(); //vortex built in encoder
            m_driveEncoder.setPosition(0); 
            m_driveMotorConfig
                .smartCurrentLimit(DriveConst.kMaxDriveAmps)
                .idleMode(IdleMode.kBrake)
                .closedLoopRampRate(DriveConst.kClosedLoopRampRate);
            m_driveMotorConfig.encoder
                .positionConversionFactor(DriveConst.rotationsToMetersScaler)
                .velocityConversionFactor(DriveConst.rpmToVelocityScaler);
            m_MaxMotionConfig = new MAXMotionConfig();
            m_MaxMotionConfig
                .maxAcceleration(DriveConst.kMaxModuleAccel)
                .cruiseVelocity(DriveConst.kMaxModuleSpeed)
                .allowedProfileError(DriveConst.kVelocityTolerance);
            m_driveController = m_driveMotor.getClosedLoopController();

            m_driveMotorConfig.closedLoop
                .pid(DriveConst.kP, DriveConst.kI, DriveConst.kD)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1,1)
                .maxMotion
                    .apply(m_MaxMotionConfig);
            m_driveMotorConfig.closedLoop
                .feedForward
                    .kS(DriveConst.kS)
                    .kV(DriveConst.kV)
                    .kA(DriveConst.kA);


            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            m_turningMotor = new SparkMax(turningMotorChannel, SparkLowLevel.MotorType.kBrushless);
            m_turningMotorConfig = new SparkMaxConfig(); 
            m_turningEncoder = m_turningMotor.getAbsoluteEncoder();

            m_turningMotorConfig.absoluteEncoder
                .inverted(false)
                .velocityConversionFactor(DriveConst.turnEncoderScaler/60)
                .positionConversionFactor(DriveConst.turnEncoderScaler);
            m_turningMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(40);
            m_turningMotorConfig.closedLoop
                .pid(.7, 0.0, 0.05)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true) //this and line below it allow for position wrapping between 0 and 2pi radians 
                .positionWrappingInputRange(0, 2*Math.PI)
                .outputRange(-1, 1);

            m_turnController = m_turningMotor.getClosedLoopController();
            m_turningMotor.configure(m_turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            System.out.println("Drive Motor CAN ID: " + driveMotorChannel + ", Initial Encoder Value: " + Math.toDegrees(m_turningEncoder.getPosition()));
        }

        /**
         * Sets the desired state for the module.
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
            desiredState.optimize(Rotation2d.fromRadians(m_turningEncoder.getPosition()));// Optimize the reference state to avoid spinning further than 90 degrees
            desiredState.cosineScale(Rotation2d.fromRadians(m_turningEncoder.getPosition()));

            m_driveController.setSetpoint(desiredState.speedMetersPerSecond, ControlType.kVelocity); //desired state gives velocity, to convert: rpm = (Velocity(in m/s) * 60)/pi*diameter(aka wheel circumference)
            m_turnController.setSetpoint(desiredState.angle.getRadians(), ControlType.kPosition);
        }
        
        public SwerveModuleState getState() {
            m_state.angle = Rotation2d.fromRadians(m_turningEncoder.getPosition());
            m_state.speedMetersPerSecond = m_driveEncoder.getVelocity();
            return m_state;
        }

        public SwerveModulePosition getPosition() {
            m_position.angle = Rotation2d.fromRadians(m_turningEncoder.getPosition());
            m_position.distanceMeters = m_driveEncoder.getPosition();
            return m_position;
        }

        /*
         * further testing (on ground) is needed for this
         */
        public void DRSMode() {
            m_DRSConfig.apply(m_driveMotorConfig);
            m_DRSConfig.smartCurrentLimit(70,20);
            m_driveMotor.configure(m_DRSConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        public void regularMode() {
            m_driveMotor.configure(m_driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        /*
         * Makes sure we can run our "turbo mode". Returns true if can run, false if can't.
         */
        public boolean tempCheck() {
            boolean coolDownMode = false;
            if(m_driveMotor.getMotorTemperature() > 70) {
                coolDownMode = true; //when we exceed the limit, we want to go into cooldown mode that doesn't change 
                                     //until we are below 60 degrees (we don't want it to be on at 69.99, off at 70, on at 69.99, etc.)
            } else if (m_driveMotor.getMotorTemperature() < 60){
                coolDownMode = false; //we can run our mode again when the motor is back down to 60 degrees. 
            }
            return !coolDownMode; //if cooldown mode is on, can't run
        }
}