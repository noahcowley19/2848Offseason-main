package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kMagazineMotorID;
import static frc.robot.Constants.ShooterConstants.kMagazinePeakVoltage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase{
    private final TalonFX magazine;

    public Magazine(){
        magazine = new TalonFX(kMagazineMotorID);
        TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
        magazineConfig.Slot0.kS = 0;
        magazineConfig.Slot0.kV = 0.12;
        magazineConfig.Slot0.kP = 0;
        magazineConfig.Slot0.kI = 0;
        magazineConfig.Slot0.kD = 0;
        magazineConfig.Slot0.kA = 0;
        magazineConfig.Voltage
            .withPeakForwardVoltage(kMagazinePeakVoltage);
            magazineConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            magazine.getConfigurator().apply(magazineConfig);
    }
    public void check(double)






        
            










    }







    
}
