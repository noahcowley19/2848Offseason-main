package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kTurretMotorID;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestTurret extends SubsystemBase{
     private final TalonFX m_Turret = new TalonFX(kTurretMotorID);
     private double kP = 5;
     private double target = Math.PI;
     private double position = 0;
     private double error = 0;
     private double dcycOutput = 0;


     public TestTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
     }
     public void periodic(){

        position = m_Turret.getPosition().getValue().magnitude();
        error = target - position;
        dcycOutput = error*kP;




     }


        








     }

    
    



