package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kTurretCANcoderID;
import static frc.robot.Constants.ShooterConstants.kTurretMotorID;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestTurret extends SubsystemBase{
     private final TalonFX m_Turret = new TalonFX(kTurretMotorID);
    private final CANcoder m_Encoder = new CANcoder(kTurretCANcoderID);
    
     private double kP = (2*180)/Math.PI;
     private double target = Math.PI/2;
     private double position = 0;
     private double error = 0;
     private double dcycOutput = 0;
     private double offset = -0.327393;



     public TestTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = offset;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_Encoder.getConfigurator().apply(CANconfig);
        m_Turret.getConfigurator().apply(config);
     }

     @Override
     public void periodic(){

        //position = m_Turret.getPosition().getValue().magnitude();

       position = m_Encoder.getAbsolutePosition().getValueAsDouble() * Math.PI*2;
        
        error = target - position;

        dcycOutput = error*kP;

        if(dcycOutput>100)
            dcycOutput = 100;
        else if(dcycOutput<-100)
            dcycOutput = -100;

        m_Turret.set(dcycOutput/100);


        
        SmartDashboard.putData(this);


     }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("Example!", ()-> "This is what it looks like", null);
        builder.addDoubleProperty("Turret Error",()-> error, null);
        builder.addDoubleProperty("TurretDuty Cycle",()-> dcycOutput, null);
        builder.addDoubleProperty("Turret Position",()-> position, null);


        
    }

        








     }

    
    



