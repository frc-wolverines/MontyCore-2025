package team5274.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public class Vision {
    private static Vision _instance;

    private Pose2d visionPose;
    
    public static Vision getInstance() {
        if(_instance == null) _instance = new Vision();
        return _instance;
    }
}
