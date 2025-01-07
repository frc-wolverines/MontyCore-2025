package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Drive {
    public static enum DriveControlState {
        OPEN_LOOP,
        HEADING,
        TRAJECTORY
    }

    private static Drive _instance;

    public static Drive get() {
        if(_instance == null) _instance = new Drive();
        return _instance;
    }
}
