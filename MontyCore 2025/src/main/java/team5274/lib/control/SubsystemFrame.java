package team5274.lib.control;

public interface SubsystemFrame {

    /**Pushes telemetry data to NetworkTables*/
    public abstract void sendTelemetry();

    /**Reset Subsystem sensors to zero*/
    public abstract void zeroSensors();

    /**Stop all hardware output*/
    public abstract void stop();
}