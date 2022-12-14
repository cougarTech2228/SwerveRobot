// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    // By default we use a Pigeon for our gyroscope. But if you use another
    // gyroscope, like a NavX, you can change this.
    // The important thing about how you configure your gyroscope is that rotating
    // the robot counter-clockwise should
    // cause the angle reading to increase until it wraps back over to zero.
    private final PigeonIMU m_pigeon = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final TalonFX m_frontLeftDriveMotor = new TalonFX(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID);
    private final TalonFX m_frontRightDriveMotor = new TalonFX(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID);
    private final TalonFX m_backLeftDriveMotor = new TalonFX(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID);
    private final TalonFX m_backRightDriveMotor = new TalonFX(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID);

    private final TalonFX m_frontLeftSteerMotor = new TalonFX(Constants.FRONT_LEFT_MODULE_STEER_MOTOR_ID);
    private final TalonFX m_frontRightSteerMotor = new TalonFX(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR_ID);
    private final TalonFX m_backLeftSteerMotor = new TalonFX(Constants.BACK_LEFT_MODULE_STEER_MOTOR_ID);
    private final TalonFX m_backRightSteerMotor = new TalonFX(Constants.BACK_RIGHT_MODULE_STEER_MOTOR_ID);

    // private final CANCoder m_frontLeftCANCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_MOTOR_ID);
    // private final CANCoder m_frontRightCANCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_MOTOR_ID);
    // private final CANCoder m_backLeftCANCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_MOTOR_ID);
    // private final CANCoder m_backRightCANCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_MOTOR_ID);

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private static final double CLOSED_LOOP_RAMP = 0.5; // seconds
    private static final double VOLTAGE_COMPENSATION_SATURATION = 11.0; // volts

    private static final double INITIAL_INPUT_ADJUSTMENT = 0.25;

    private NetworkTableEntry m_forwardAdjustmentTableEntry;
    private NetworkTableEntry m_sidewaysAdjustmentTableEntry;
    private NetworkTableEntry m_rotationalAdjustmentTableEntry;

    private static final boolean INITIAL_FIELD_ORIENTED_SETTING = true;

    private NetworkTableEntry m_isFieldOrientedTableEntry;

    private SwerveDriveOdometry m_odometry;

    private boolean doingTeleOpAuto;

    public DrivetrainSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of
                // the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 3)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4iSwerveModuleHelper.GearRatio.L2,
                // This is the ID of the drive motor
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                // This is the ID of the steer motor
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                // This is the ID of the steer encoder
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER_ID,
                // This is how much the steer encoder is offset from true zero (In our case,
                // zero is facing straight forward)
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

        // We will do the same for the other modules
        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 3)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 3)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR_ID,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER_ID,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET);

        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 3)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR_ID,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER_ID,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

        configureDriveMotor(m_frontLeftDriveMotor);
        configureDriveMotor(m_frontRightDriveMotor);
        configureDriveMotor(m_backLeftDriveMotor);
        configureDriveMotor(m_backRightDriveMotor);

        // configureSteerSensor(m_frontLeftCANCoder);
        // configureSteerSensor(m_frontRightCANCoder);
        // configureSteerSensor(m_backLeftCANCoder);
        // configureSteerSensor(m_backRightCANCoder);

        doingTeleOpAuto = false;
                       
        zeroGyroscope();

        m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0));

        // Add widgets to adjust controller input values and robot-v-field orientation
        m_forwardAdjustmentTableEntry = tab.add("Forward Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(0, 3)
                .getEntry();

        m_sidewaysAdjustmentTableEntry = tab.add("Sideways Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(2, 3)
                .getEntry();

        m_rotationalAdjustmentTableEntry = tab.add("Rotational Adj", INITIAL_INPUT_ADJUSTMENT)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", INITIAL_INPUT_ADJUSTMENT, "max", 1))
                .withSize(2, 1)
                .withPosition(4, 3)
                .getEntry();

        m_isFieldOrientedTableEntry = tab.add("Field Oriented?", INITIAL_FIELD_ORIENTED_SETTING)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withSize(2, 1)
                .withPosition(6, 3)
                .getEntry();
    }

    private void configureDriveMotor(TalonFX motor) {
        motor.configClosedloopRamp(CLOSED_LOOP_RAMP);
        motor.configVoltageCompSaturation(VOLTAGE_COMPENSATION_SATURATION);
        motor.enableVoltageCompensation(true);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    }

    // KAS - this fails every time and you can't configure the CANCoders via Pheonix Tuner, WTF?
    // private void configureSteerSensor(CANCoder sensor) {
    //     if (sensor.configGetSensorInitializationStrategy(1000).value == SensorInitializationStrategy.BootToZero.value) {
    //         if (sensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 1000).value != ErrorCode.OK.value) {
    //             System.out.println("ERROR: Couldn't set the initialization strategy");
    //         }
    //     }
    // }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        m_pigeon.setFusedHeading(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void stopMotors() {
        System.out.println("stopMotors");
        drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    public void setMotorsToCoast() {
        System.out.println("setMotorsToCoast");
        m_frontLeftDriveMotor.setNeutralMode(NeutralMode.Coast);
        m_frontRightDriveMotor.setNeutralMode(NeutralMode.Coast);
        m_backLeftDriveMotor.setNeutralMode(NeutralMode.Coast);
        m_backRightDriveMotor.setNeutralMode(NeutralMode.Coast);

        m_frontLeftSteerMotor.setNeutralMode(NeutralMode.Coast);
        m_frontRightSteerMotor.setNeutralMode(NeutralMode.Coast);
        m_backLeftSteerMotor.setNeutralMode(NeutralMode.Coast);
        m_backRightSteerMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setMotorsToBrake() {
        System.out.println("setMotorsToBrake");
        m_frontLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_frontRightDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_backLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
        m_backRightDriveMotor.setNeutralMode(NeutralMode.Brake);

        m_frontLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
        m_frontRightSteerMotor.setNeutralMode(NeutralMode.Brake);
        m_backLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
        m_backRightSteerMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getForwardAdjustment() {
        return m_forwardAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    public double getSidewaysAdjustment() {
        return m_sidewaysAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    public boolean getIsFieldOrientedSetting() {
        return m_isFieldOrientedTableEntry.getBoolean(INITIAL_FIELD_ORIENTED_SETTING);
    }

    public double getRotationalAdjustmennt() {
        return m_rotationalAdjustmentTableEntry.getDouble(INITIAL_INPUT_ADJUSTMENT);
    }

    // This is an attempt to get rid of the "dead wheel" issue when the CANCoder
    // isn't initialized properly. 
    public void primeDrivetrain() {
        System.out.println("primeDrivetrain");
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        drive(chassisSpeeds);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                states[3].angle.getRadians());
    }

    public double getYaw() {
        return Math.IEEEremainder(m_pigeon.getYaw(), 360.0d);
    }

    public void setDoingTeleOpAuto(boolean doingTeleOpAuto) {
        this.doingTeleOpAuto = doingTeleOpAuto;
    }

    public SwerveDriveOdometry getOdometry() {
        return m_odometry;
    } 

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        m_odometry.update(Rotation2d.fromDegrees(getYaw()), states);

        if(!doingTeleOpAuto) {
            setModuleStates(states);
        }
    }
}
