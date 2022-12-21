// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5; // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.51; // Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 61; // Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 47; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 48; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 49; // Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(305); // FIXME Measure and set front
                                                                                        // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 56; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 57; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 58; // Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(163.38); // FIXME Measure and set front
                                                                                         // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 50; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 51; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 52; // Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(124.22); // FIXME Measure and set back
                                                                                       // left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 53; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 54; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 55; // Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(66.04); // FIXME Measure and set back
                                                                                       // right steer offset

    public static final double APRIL_TAG_COARSE_FORWARD_DRIVE_SPEED_IN_M_PER_S = 0.8;
    public static final double APRIL_TAG_FINE_FORWARD_DRIVE_SPEED_IN_M_PER_S = 0.5;
    public static final double APRIL_TAG_SIMULTANEOUS_FORWARD_DRIVE_SPEED_IN_M_PER_S = 0.5;

    public static final double APRIL_TAG_COARSE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S = 0.5;
    public static final double APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S = 0.4;

    public static final double APRIL_TAG_COARSE_TURN_SPEED_IN_M_PER_S = 0.4;
    public static final double APRIL_TAG_FINE_TURN_SPEED_IN_M_PER_S = 0.4;
}
