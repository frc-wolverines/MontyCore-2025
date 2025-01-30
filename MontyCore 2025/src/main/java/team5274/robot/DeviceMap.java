package team5274.robot;

import team5274.lib.hardware.drivers.DeviceId;
import team5274.lib.hardware.drivers.DeviceId.Bus;

public class DeviceMap {

    public static class ElevatorMap {
        public static final DeviceId kMasterMotorId = new DeviceId(0, Bus.CAN); //Configure
        public static final DeviceId kSlaveMotorId = new DeviceId(0, Bus.CAN); //Configure
    }

}
