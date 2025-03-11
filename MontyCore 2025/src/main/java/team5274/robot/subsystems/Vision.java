package team5274.robot.subsystems;

public class Vision {
    private static Vision _instance;
    
    public static Vision getInstance() {
        if(_instance == null) _instance = new Vision();
        return _instance;
    }
}
