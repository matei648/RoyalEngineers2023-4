package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class swerveModule {

    DcMotorEx motor;
    CRServo servo;
    absoluteAnalogEncoder encoder;

    public static double target = 0;
    boolean motorDirection = true;
    public static double P = 0.018, I = 0, D = 0.0007, F = 0;

    public static double tolerance = 2;
    PIDFController angleController = new PIDFController(P, I, D, F);
    Telemetry telemetrieba;
    public swerveModule(DcMotorEx motor, CRServo servo, absoluteAnalogEncoder encoder, Telemetry telemetry) {
        this.motor = motor; this.servo = servo; this.encoder = encoder;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetrieba = telemetry;
        angleController.setTolerance(tolerance);
    }

    public void drive(double motorPower, double servoAngle) {

        target = servoAngle + 180;
        target = 360 - target;

        double current = encoder.getCurrentPosition();
        double error;
        error = getGigaTarget(target, current);
        double power = angleController.calculate(0, error);
        if ( Math.abs(error) < tolerance )
            power = 0;

        if ( motorPower < 0.03 )
            servo.setPower(0.0d);
        else
        servo.setPower(power);


        telemetrieba.addData("Pow: ", power);
        telemetrieba.addData("PowMot: ", motorPower);
        telemetrieba.addData("Target ", target);
        telemetrieba.addData("Pos ", current);
        telemetrieba.addData("Eroor: ", error);

        if(motorDirection == true)
            motor.setPower(motorPower);
        else
            motor.setPower(-motorPower);

    }

    private double getGigaTarget(double target, double current){

        double target1 = target, target2 = (target+180)%360;
        double errorpoz = 0.0d, errorneg = 0.0d;
        double errorpoz1 = 0.0d, errorneg1 = 0.0d;

        double output = 0.0d;

        if ( current >= target1){
            errorpoz = (360 - current) + target1;
            errorneg = current - target1;
        }
        else{
            errorpoz = target1 - current;
            errorneg = current + ( 360 - target1 );
        }

        if ( current >= target2){
            errorpoz1 = (360 - current) + target2;
            errorneg1 = current - target2;
        }
        else{
            errorpoz1 = target2 - current;
            errorneg1 = current + ( 360 - target2 );
        }

        errorpoz*=-1;
        errorpoz1*=-1;

        double min = 1000;
        min = Math.min(Math.abs(errorpoz), min);
        min = Math.min(Math.abs(errorpoz1), min);
        min = Math.min(Math.abs(errorneg), min);
        min = Math.min(Math.abs(errorneg1), min);


        if ( Math.abs(errorpoz) == min ) {
            motorDirection = true;
            return errorpoz;

        }

        if ( Math.abs(errorneg) == min ) {
            motorDirection = true;
            return errorneg;
        }

        if ( Math.abs(errorpoz1) == min ) {
            motorDirection = false;
            return errorpoz1;
        }

        if ( Math.abs(errorneg1) == min ) {
            motorDirection = false;
            return errorneg1;
        }
        return output;

    }
}