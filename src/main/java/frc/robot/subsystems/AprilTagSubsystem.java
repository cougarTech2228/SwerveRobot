package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagSubsystem extends SubsystemBase {

    static NetworkTable m_aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTag");
    static NetworkTableEntry m_pitch = m_aprilTagTable.getEntry("Pitch");
    static NetworkTableEntry m_tx = m_aprilTagTable.getEntry("TX");
    static NetworkTableEntry m_tz = m_aprilTagTable.getEntry("TZ");
    static NetworkTableEntry m_tagID = m_aprilTagTable.getEntry("Tag ID");

    public AprilTagSubsystem() {
    }

    public double getPitch() {
        return m_pitch.getDouble(2228);
    }

    public double getTX() {
        return m_tx.getDouble(2228);
    }

    public double getTZ() {
        return m_tz.getDouble(2228);
    }

    public double getTagID() {
       return m_tagID.getDouble(2228);
    }

    public long getLastChanged() {
        return m_tagID.getInfo().last_change;
    }

    @Override
    public void periodic() {
        //System.out.println("TZ: " + getTZ() + " | TagID: " + getTagID());
        //System.out.println("TagID last_change: " + m_tagID.getInfo().last_change);
    }
}