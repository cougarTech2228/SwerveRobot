package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AprilTagSubsystem;

public class DockWithAprilTagPIDCommand extends CommandBase {

    private static double kDt = 0.02;

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final AprilTagSubsystem m_aprilTagSubsystem;
    private final double m_aprilTagId;

    private static final double MAX_FORWARD_DOCKING_VELOCITY = .75; // meters/second
    private static final double MAX_FORWARD_DOCKING_ACCELERATION = 0.25; // meters/second**2

    private final TrapezoidProfile.Constraints m_forwardConstraints = 
        new TrapezoidProfile.Constraints(MAX_FORWARD_DOCKING_VELOCITY, MAX_FORWARD_DOCKING_ACCELERATION);

    private final double FORWARD_P = 0.5; 
    private final double FORWARD_D = 0.0; 
    private final ProfiledPIDController m_forwardController =
        new ProfiledPIDController(FORWARD_P, 0.0, FORWARD_D, m_forwardConstraints, kDt);

    private static final double MAX_SIDEWAYS_DOCKING_VELOCITY = .75; // meters/second
    private static final double MAX_SIDEWAYS_DOCKING_ACCELERATION = 0.25; // meters/second**2

    private final TrapezoidProfile.Constraints m_sidewaysConstraints = 
        new TrapezoidProfile.Constraints(MAX_SIDEWAYS_DOCKING_VELOCITY, MAX_SIDEWAYS_DOCKING_ACCELERATION);
    
    private final double SIDEWAYS_P = 0.5; 
    private final double SIDEWAYS_D = 0.0;
    private final ProfiledPIDController m_sidewaysController =
        new ProfiledPIDController(SIDEWAYS_P, 0.0, SIDEWAYS_D, m_sidewaysConstraints, kDt);

    private static final double DOCKING_DISTANCE_GOAL_METERS = 0.75;

    private static final double MIN_FORWARD_VELOCITY = 0.2;
    private static final double MIN_SIDEWAYS_VELOCITY = 0.2;

    private long m_lastChanged = 0;
    private double m_start_time = 0;

    public DockWithAprilTagPIDCommand(DrivetrainSubsystem drivetrainSubsystem,
            AprilTagSubsystem aprilTagSubsystem,
            double aprilTagId) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_aprilTagSubsystem = aprilTagSubsystem;
        this.m_aprilTagId = aprilTagId;

        addRequirements(drivetrainSubsystem, aprilTagSubsystem);
    }

    @Override
    public void initialize() {
        if (m_aprilTagSubsystem.getTagID() == m_aprilTagId) {
            m_lastChanged = m_aprilTagSubsystem.getLastChanged();
            m_start_time = Timer.getFPGATimestamp();

            m_forwardController.setGoal(DOCKING_DISTANCE_GOAL_METERS);
            m_sidewaysController.setGoal(0.0);
        } 
    }

    @Override
    public boolean isFinished() {
        if ((m_aprilTagSubsystem.getTagID() == m_aprilTagId) &&
            (m_aprilTagSubsystem.getLastChanged() == m_lastChanged)) {

            double distanceToTarget = m_aprilTagSubsystem.getTZ();
            double offsetTargetDistance = m_aprilTagSubsystem.getTX();

            double forwardSpeed = -m_forwardController.calculate(distanceToTarget);
            double sidewaysSpeed = m_sidewaysController.calculate(offsetTargetDistance);

            double forwardVelocity = forwardSpeed * MAX_FORWARD_DOCKING_VELOCITY;
            double sidewaysVelocity = sidewaysSpeed * MAX_SIDEWAYS_DOCKING_VELOCITY;

            // Need to ensure minimum velocities that are high enough to move the robot
            if (forwardVelocity < MIN_FORWARD_VELOCITY) {
                forwardVelocity = MIN_FORWARD_VELOCITY;
            }

            if ((sidewaysVelocity > 0.0) && (sidewaysVelocity < MIN_SIDEWAYS_VELOCITY)) {
                sidewaysVelocity = MIN_SIDEWAYS_VELOCITY;
            }

            if ((sidewaysVelocity < 0.0) && (sidewaysVelocity > -MIN_SIDEWAYS_VELOCITY)) {
                sidewaysVelocity = -MIN_SIDEWAYS_VELOCITY;
            }

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                forwardVelocity,
                sidewaysVelocity,
                0.0);

            //System.out.println("F: " + forwardVelocity + " S: " + sidewaysVelocity);

            m_drivetrainSubsystem.drive(chassisSpeeds);

            // Check to see if we're within docking distance
            if (distanceToTarget < DOCKING_DISTANCE_GOAL_METERS) {
                System.out.println("Docked with target. Yipee!!!!!");
                System.out.println("Command completed in " + (Timer.getFPGATimestamp() - m_start_time) + " seconds");
                return true;
            } else {
                return false;
            }
        } else {
            System.out.println("AprilTag with ID = " + m_aprilTagId + " not detected. Stopping command.");
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_lastChanged = 0;
        m_drivetrainSubsystem.stopMotors();
    }
}
