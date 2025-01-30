package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PincerConstants;

public class Pincer {

    private static Pincer _instance;

    public static Pincer get() {
        if(_instance == null) _instance = new Pincer();
        return _instance;
    }

    private Pincer() {

    }
}
