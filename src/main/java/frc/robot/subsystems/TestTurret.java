package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.kMaxTurretRad;
import static frc.robot.Constants.ShooterConstants.kMinTurretRad;
import static frc.robot.Constants.ShooterConstants.kTurretCANcoderID;
import static frc.robot.Constants.ShooterConstants.kTurretMotorID;
import static frc.robot.RangerHelpers.retryConfigApply;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestTurret extends SubsystemBase{
     private final TalonFX m_Turret = new TalonFX(kTurretMotorID);
    private final CANcoder m_Encoder = new CANcoder(kTurretCANcoderID);
    
     private double kP = (.5*180)/Math.PI;
     private double target = Math.PI/2;
     private double position = 0;
     private double error = 0;
     private double dcycOutput = 0;
     private double offset = 0.327393;
     private double kTestVolts = 2;
     private boolean closedLoop = true;

     private double tempTarget = 0;

     public TestTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        CANcoderConfiguration CANconfig = new CANcoderConfiguration();
        CANconfig.MagnetSensor.MagnetOffset = offset;
        //config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        retryConfigApply(()->m_Encoder.getConfigurator().apply(CANconfig));
     }
     public void periodic(){

      SmartDashboard.putData(this);

        //position = m_Turret.getPosition().getValue().magnitude();

       position = m_Encoder.getAbsolutePosition().getValueAsDouble() * Math.PI*2;
        
        error = target - position;

        dcycOutput = error*kP;

        if(dcycOutput>100)
            dcycOutput = 100;
        else if(dcycOutput<-100)
            dcycOutput = -100;

        // skip closed loop while runVolts is driving the motor
        if(closedLoop)
            m_Turret.set(dcycOutput/100);
     }
     public void setTarget(double radians)
     {
        target = radians; //MathUtil.clamp(radians, kMinTurretRad, kMaxTurretRad);
     }
     public Command goTo(double radians)
     {
        return runOnce(() -> setTarget(radians));
     }
     public Command goToDeg(double degrees)
     {
        return goTo(Math.toRadians(degrees));
     }
     public Command runVolts()
     {
        return startEnd(
            () -> {
                closedLoop = false;
                m_Turret.setVoltage(kTestVolts);
            },
            () -> {
                setTarget(position);
                closedLoop = true;
            });
     }
     public boolean atTarget()
     {
        return Math.abs(error) < Math.toRadians(2);
     }



     @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Turret Angle",
        ()->target,
        value -> setTarget(value));

        builder.addDoubleProperty("Measured Angle",
        ()->m_Encoder.getAbsolutePosition().getValueAsDouble(),
        null);

        



    }
        








     }

    
    



