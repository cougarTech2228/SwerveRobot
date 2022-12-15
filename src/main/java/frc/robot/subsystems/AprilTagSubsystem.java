package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

public class AprilTagSubsystem extends SubsystemBase {

    static NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTag");
    static NetworkTableEntry m_pitch = m_aprilTagTable.getEntry("Pitch"); 
    static NetworkTableEntry m_roll = m_aprilTagTable.getEntry("Roll"); 
    static NetworkTableEntry m_tx = m_aprilTagTable.getEntry("TX"); 
    static NetworkTableEntry m_ty = m_aprilTagTable.getEntry("TY"); 
    static NetworkTableEntry m_tz = m_aprilTagTable.getEntry("TZ"); 
    static NetworkTableEntry m_tagID = m_aprilTagTable.getEntry("Tag ID"); 
    static NetworkTableEntry m_yaw = m_aprilTagTable.getEntry("Yaw"); 

    public AprilTagSubsystem(){}
    public double getPitch(){
        return m_pitch.getDouble(2228);
    }
    public double getRoll(){
        return m_roll.getDouble(2228);
    }
    public double getTX(){
        return m_tx.getDouble(2228);
    }
    public double getTY(){
        return m_ty.getDouble(2228);
    }
    public double getTZ(){
        return m_tz.getDouble(2228);
    }
    public double getTagID(){
        return m_tagID.getDouble(2228);
    }
    public double getYaw(){
        return m_yaw.getDouble(2228);
    }

    @Override
    public void periodic() {
        System.out.println("TZ: " + getTZ() + " | TagID: " + getTagID());
    }
}