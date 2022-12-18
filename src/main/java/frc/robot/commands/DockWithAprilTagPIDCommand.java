package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;

public class DockWithAprilTagPIDCommand extends CommandBase {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final AprilTagSubsystem m_aprilTagSubsystem;
    private final double m_aprilTagId;

    private final double LINEAR_P = 0.18;
    private final double LINEAR_D = 0.0;
    private PIDController m_forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    private final double SIDEWAYS_P = 0.12;
    private final double SIDEWAYS_D = 0.0;
    private PIDController m_sidewaysController = new PIDController(SIDEWAYS_P, 0, SIDEWAYS_D);

    private static final double DOCKING_DISTANCE_GOAL_METERS = 0.75;

    public DockWithAprilTagPIDCommand(DrivetrainSubsystem drivetrainSubsystem,
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

            double forwardSpeed = -m_forwardController.calculate(distanceToTarget, DOCKING_DISTANCE_GOAL_METERS);
            double sidewaysSpeed = m_sidewaysController.calculate(offsetTargetDistance, 0);

            double forwardVelocity = forwardSpeed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            double sidewaysVelocity = sidewaysSpeed * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;

            // Need to ensure minimum velocities that are high enough to move the robot
            if (forwardVelocity < 0.2) forwardVelocity = 0.2;
            if ((sidewaysVelocity > 0.0) && (sidewaysVelocity < 0.2)) sidewaysVelocity = 0.2;
            if ((sidewaysVelocity < 0.0) && (sidewaysVelocity > -0.2)) sidewaysVelocity = -0.2;

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                forwardVelocity,
                sidewaysVelocity,
                0.0);

            //System.out.println("F: " + forwardVelocity + " S: " + sidewaysVelocity);

            m_drivetrainSubsystem.drive(chassisSpeeds);

            // Check to see if we're within docking distance
            if (distanceToTarget < DOCKING_DISTANCE_GOAL_METERS) {
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
