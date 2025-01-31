// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5274.lib.hardware.drivers;

/** A class that stores a device's id used in various busses */
public class DeviceId {
    public static enum Bus {
        CAN,
        DIO,
        PWM,
        AIN
    }

    private final int deviceId;
    private final Bus bus;

    public DeviceId(int deviceId, Bus bus) {
        this.deviceId = deviceId;
        this.bus = bus;
    }

    public int getDeviceId() {
        return deviceId;
    }

    public boolean equals(DeviceId other) {
        return other.deviceId == deviceId && other.bus == bus;
    }
}