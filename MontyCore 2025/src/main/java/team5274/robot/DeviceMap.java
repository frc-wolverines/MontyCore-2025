package team5274.robot;

import team5274.lib.hardware.drivers.DeviceId;
import team5274.lib.hardware.drivers.DeviceId.Bus;

public class DeviceMap {

    //We reserve CAN IDs 0 and 1 for the RoboRIO and PDH

    public static class DriveMap {
        public static final DeviceId kLeftFrontDriveMotorId = new DeviceId(2, Bus.CAN); 
        public static final DeviceId kLeftFrontPivotMotorId = new DeviceId(3, Bus.CAN); 
        public static final DeviceId kLeftFrontModuleEncoderId = new DeviceId(4, Bus.CAN); 

        public static final DeviceId kRightFrontDriveMotorId = new DeviceId(5, Bus.CAN); 
        public static final DeviceId kRightFrontPivotMotorId = new DeviceId(6, Bus.CAN); 
        public static final DeviceId kRightFronModuleEncoderId = new DeviceId(7, Bus.CAN); 

        public static final DeviceId kLeftBackDriveMotorId = new DeviceId(8, Bus.CAN); 
        public static final DeviceId kLeftBackPivotMotorId = new DeviceId(9, Bus.CAN); 
        public static final DeviceId kLeftBackModuleEncoderId = new DeviceId(10, Bus.CAN); 

        public static final DeviceId kRightBackDriveMotorId = new DeviceId(11, Bus.CAN); 
        public static final DeviceId kRightBackPivotMotorId = new DeviceId(12, Bus.CAN); 
        public static final DeviceId kRightBackModuleEncoderId = new DeviceId(13, Bus.CAN); 
    }

    public static class ElevatorPivotMap {
        public static final DeviceId kMasterMotorId = new DeviceId(14, Bus.CAN); 
        public static final DeviceId kSlaveMotorId = new DeviceId(15, Bus.CAN); 
        public static final DeviceId kEncoderId = new DeviceId(0, Bus.DIO); 
    }

    public static class ElevatorMap {
        public static final DeviceId kMasterMotorId = new DeviceId(16, Bus.CAN); 
        public static final DeviceId kSlaveMotorId = new DeviceId(17, Bus.CAN); 
    }

    public static class ArmMap {
        public static final DeviceId kPinionMotorId = new DeviceId(18, Bus.CAN); 
        public static final DeviceId kWristMotorId = new DeviceId(19, Bus.CAN);
        public static final DeviceId kEncoderId = new DeviceId(1, Bus.DIO); 
    }

    public static class PincerMap {
        public static final DeviceId kIntakeMotorId = new DeviceId(20, Bus.CAN); 
    }
}
