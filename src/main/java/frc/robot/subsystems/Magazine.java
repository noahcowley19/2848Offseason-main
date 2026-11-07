package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.kMagazineMotorID;
import static frc.robot.Constants.ShooterConstants.kMagazinePeakVoltage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Magazine extends SubsystemBase{

    final private TalonFX magazine;
    private double magazineSpeed = 0;

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
            .withPeakForwardVoltage(Volts.of(kMagazinePeakVoltage));
        magazineConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        magazine.getConfigurator().apply(magazineConfig);
    }

    public void periodic()
    {
        magazine.set(magazineSpeed);
        double correction = RobotContainer.getTurret().getCorrection();
        double FlyWheelError = RobotContainer.getFlywheel().getError();
        if(correction<=5 && FlyWheelError<=1)
        {
            magazine.set(magazineSpeed);
        }


    }
    public void setSpeed(double speed)
    {
        magazineSpeed = speed;
    }
    public void stop()
    {
        magazineSpeed = 0;
    }

   
    
}
