package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.HopperConstants.kForwardBeltMotorID;
import static frc.robot.Constants.HopperConstants.kSidewaysBeltMotorID;

import java.lang.annotation.Target;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transition extends SubsystemBase{


    private final TalonFX sidewaysMotor;
    private final TalonFX forwardMotor;
    private final double defaultSpeed = 0;

    private double beltSpeed = .1;


    public Transition()
    {

        this.register();
        sidewaysMotor = new TalonFX(kSidewaysBeltMotorID);
        forwardMotor = new TalonFX(kForwardBeltMotorID);
        TalonFXConfiguration beltConfig = new TalonFXConfiguration();

        beltConfig.Slot0.kS = 0;
        beltConfig.Slot0.kV = 0.12;
        beltConfig.Slot0.kP = 0;
        beltConfig.Slot0.kI = 0;
        beltConfig.Slot0.kD = 0;
        beltConfig.Slot0.kA = 0;

        beltConfig.Voltage
            .withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));
        beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        sidewaysMotor.getConfigurator().apply(beltConfig);
        forwardMotor.getConfigurator().apply(beltConfig);
        sidewaysMotor.setControl(new Follower(forwardMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setSpeed(double speed)
    {

        beltSpeed = speed;
        
    }
    public void stop()
    {
        beltSpeed=0;
    }

    public void periodic()
    {
        forwardMotor.set(beltSpeed);
        SmartDashboard.putData(this);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Velocity Output", () -> forwardMotor.getVelocity().getValueAsDouble(), null);
       builder.addDoubleProperty("Voltage", () -> forwardMotor.getMotorVoltage().getValueAsDouble(), null);


       builder.addDoubleProperty("Speed",
        ()->defaultSpeed,
        value -> setSpeed(value));

    }
}
    
