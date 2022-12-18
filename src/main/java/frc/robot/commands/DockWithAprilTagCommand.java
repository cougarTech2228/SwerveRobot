package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagSubsystem;

public class DockWithAprilTagCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final AprilTagSubsystem m_aprilTagSubsystem;
    private final double m_aprilTagId;

    private final ChassisSpeeds m_driveForwardCoarseChassisSpeeds = new ChassisSpeeds(
            Constants.APRIL_TAG_COARSE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
            0.0,
            0.0);

    private final ChassisSpeeds m_driveForwardFineChassisSpeeds = new ChassisSpeeds(
            Constants.APRIL_TAG_FINE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
            0.0,
            0.0);

    // private final ChassisSpeeds m_driveLeftCoarseChassisSpeeds =
    // new ChassisSpeeds(Constants.APRIL_TAG_COARSE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
    // Constants.APRIL_TAG_COARSE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
    // 0.0);

    // private final ChassisSpeeds m_driveLeftFineChassisSpeeds =
    // new ChassisSpeeds(Constants.APRIL_TAG_FINE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
    // Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
    // 0.0);

    // private final ChassisSpeeds m_driveRightCoarseChassisSpeeds =
    // new ChassisSpeeds(Constants.APRIL_TAG_COARSE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
    // -Constants.APRIL_TAG_COARSE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
    // 0.0);

    // private final ChassisSpeeds m_driveRightFineChassisSpeeds =
    // new ChassisSpeeds(Constants.APRIL_TAG_FINE_FORWARD_DRIVE_SPEED_IN_M_PER_S,
    // -Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
    // 0.0);

    private final ChassisSpeeds m_driveLeftCoarseChassisSpeeds = new ChassisSpeeds(
            Constants.APRIL_TAG_SIMULTANEOUS_FORWARD_DRIVE_SPEED_IN_M_PER_S,
            Constants.APRIL_TAG_COARSE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            0.0);

    private final ChassisSpeeds m_driveLeftFineChassisSpeeds = new ChassisSpeeds(
            0.0,
            Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            0.0);

    private final ChassisSpeeds m_driveRightCoarseChassisSpeeds = new ChassisSpeeds(
            Constants.APRIL_TAG_SIMULTANEOUS_FORWARD_DRIVE_SPEED_IN_M_PER_S,
            -Constants.APRIL_TAG_COARSE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            0.0);

    private final ChassisSpeeds m_driveRightFineChassisSpeeds = new ChassisSpeeds(
            0.0,
            -Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            0.0);

    private final ChassisSpeeds m_turnRightCoarseChassisSpeeds = new ChassisSpeeds(
            0.0,
            Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            -Constants.APRIL_TAG_COARSE_TURN_SPEED_IN_M_PER_S);

    private final ChassisSpeeds m_turnRightFineChassisSpeeds = new ChassisSpeeds(0.0,
            Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            -Constants.APRIL_TAG_FINE_TURN_SPEED_IN_M_PER_S);

    private final ChassisSpeeds m_turnLeftCoarseChassisSpeeds = new ChassisSpeeds(
            0.0,
            -Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            Constants.APRIL_TAG_COARSE_TURN_SPEED_IN_M_PER_S);

    private final ChassisSpeeds m_turnLeftFineChassisSpeeds = new ChassisSpeeds(0.0,
            -Constants.APRIL_TAG_FINE_SIDEWAYS_DRIVE_SPEED_IN_M_PER_S,
            Constants.APRIL_TAG_FINE_TURN_SPEED_IN_M_PER_S);

    private static final double COARSE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS = 0.15;
    private static final double FINE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS = 0.08;

    private static final double DOCKING_DISTANCE_METERS = 0.75;

    private static final double FINE_COARSE_ADJUSTMENT_DISTANCE_METERS = 1.25;

    private static final double COARSE_OFF_ANGLE_PITCH_CORRECTION = 10.0;
    private static final double FINE_OFF_ANGLE_PITCH_CORRECTION = 6.0;

    public DockWithAprilTagCommand(DrivetrainSubsystem drivetrainSubsystem,
            AprilTagSubsystem aprilTagSubsystem,
            double aprilTagId) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_aprilTagSubsystem = aprilTagSubsystem;
        this.m_aprilTagId = aprilTagId;

        addRequirements(drivetrainSubsystem, aprilTagSubsystem);
    }

    @Override
    public boolean isFinished() {
        if (m_aprilTagSubsystem.getTagID() == m_aprilTagId) {

            double distanceToTarget = m_aprilTagSubsystem.getTZ();
            double offsetTargetDistance = m_aprilTagSubsystem.getTX();
            double pitchToTarget = m_aprilTagSubsystem.getPitch();

            // Fine Adjustments
            if (distanceToTarget < FINE_COARSE_ADJUSTMENT_DISTANCE_METERS) {

                // Correct for off-center, then pitch
                if (offsetTargetDistance < -FINE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS) {
                    System.out.println("Fine correction to the left");
                    m_drivetrainSubsystem.drive(m_driveLeftFineChassisSpeeds);
                } else if (offsetTargetDistance > FINE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS) {
                    System.out.println("Fine correction to the right");
                    m_drivetrainSubsystem.drive(m_driveRightFineChassisSpeeds);
                } /*else if (pitchToTarget < -FINE_OFF_ANGLE_PITCH_CORRECTION) {
                    System.out.println("Fine pitch correction to the left");
                    m_drivetrainSubsystem.drive(m_turnLeftFineChassisSpeeds);
                } else if (pitchToTarget > FINE_OFF_ANGLE_PITCH_CORRECTION) {
                    System.out.println("Fine pitch correction to the right");
                    m_drivetrainSubsystem.drive(m_turnRightFineChassisSpeeds);
                } */else {
                    m_drivetrainSubsystem.drive(m_driveForwardFineChassisSpeeds);
                }
            } else { // Coarse Adjustments

                // Correct for off-center, then pitch
                if (offsetTargetDistance < -COARSE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS) {
                    System.out.println("Coarse correction to the left");
                    m_drivetrainSubsystem.drive(m_driveLeftCoarseChassisSpeeds);
                } else if (offsetTargetDistance > COARSE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS) {
                    System.out.println("Coarse correction to the right");
                    m_drivetrainSubsystem.drive(m_driveRightCoarseChassisSpeeds);
                } /*else if (pitchToTarget < -COARSE_OFF_ANGLE_PITCH_CORRECTION) {
                    System.out.println("Coarse pitch correction to the left");
                    m_drivetrainSubsystem.drive(m_turnLeftCoarseChassisSpeeds);
                } else if (pitchToTarget > COARSE_OFF_ANGLE_PITCH_CORRECTION) {
                    System.out.println("Coarse pitch correction to the right");
                    m_drivetrainSubsystem.drive(m_turnRightCoarseChassisSpeeds);
                } */else {
                    m_drivetrainSubsystem.drive(m_driveForwardCoarseChassisSpeeds);
                }
            }

            // Check to see if we're within docking distance
            if ((distanceToTarget < DOCKING_DISTANCE_METERS) &&
                (Math.abs(offsetTargetDistance) < FINE_LEFT_TO_RIGHT_OFFSET_CORRECTION_METERS)/* &&
                (Math.abs(pitchToTarget) < FINE_OFF_ANGLE_PITCH_CORRECTION)*/)
                {
                System.out.println("Docked with target. Yipee!!!!!");
                return true;
            } else {
                return false;
            }
        } else {
            System.out.println("Lost tag acquistion ... WTF!!!!!!!!!");
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopMotors();
    }
}
