package team5274.robot;

import team5274.lib.hardware.drivers.DeviceId;
import team5274.lib.hardware.drivers.DeviceId.Bus;
import team5274.robot.subsystems.drive.SwerveModule.SwerveModuleDeviceMap;

public class DeviceMap {

    //We reserve CAN IDs 0 and 1 for the RoboRIO and PDH

    public static class DriveMap {
        public static final SwerveModuleDeviceMap[] driveMap = {
            new SwerveModuleDeviceMap(
                new DeviceId(2, Bus.CAN), 
                new DeviceId(3, Bus.CAN), 
                new DeviceId(4, Bus.CAN)
            ),
            new SwerveModuleDeviceMap(
                new DeviceId(5, Bus.CAN), 
                new DeviceId(6, Bus.CAN), 
                new DeviceId(7, Bus.CAN)
            ),
            new SwerveModuleDeviceMap(
                new DeviceId(8, Bus.CAN), 
                new DeviceId(9, Bus.CAN), 
                new DeviceId(10, Bus.CAN)
            ),
            new SwerveModuleDeviceMap(
                new DeviceId(11, Bus.CAN), 
                new DeviceId(12, Bus.CAN), 
                new DeviceId(13, Bus.CAN)
            )
        };
    }

    public static class ElevatorPivotMap {
        public static final DeviceId kMasterMotorId = new DeviceId(14, Bus.CAN); 
        public static final DeviceId kSlaveMotorId = new DeviceId(15, Bus.CAN); 
        public static final DeviceId kEncoderId = new DeviceId(1, Bus.DIO); 
    }

    public static class ClimberClampMap {
        public static final DeviceId clampMotorId = new DeviceId(21, Bus.CAN);
    }

    public static class ElevatorMap {
        public static final DeviceId kMasterMotorId = new DeviceId(16, Bus.CAN); 
        public static final DeviceId kSlaveMotorId = new DeviceId(17, Bus.CAN); 
    }

    public static class ArmMap {
        public static final DeviceId kPinionMotorId = new DeviceId(18, Bus.CAN); 
        public static final DeviceId kWristMotorId = new DeviceId(19, Bus.CAN);
        public static final DeviceId kEncoderId = new DeviceId(2, Bus.DIO); 
    }

    public static class PincerMap {
        public static final DeviceId kIntakeMotorId = new DeviceId(20, Bus.CAN); 
    }
}
