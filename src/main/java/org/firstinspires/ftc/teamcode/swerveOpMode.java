package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.Pif;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class swerveOpMode extends LinearOpMode {

    DcMotorEx motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft;
    CRServo servoFrontRight, servoFrontLeft, servoBackRight, servoBackLeft;
    AnalogInput aencoderFrontRight, aencoderFrontLeft, aencoderBackRight, aencoderBackLeft;
    absoluteAnalogEncoder encoderFrontRight, encoderFrontLeft, encoderBackLeft, encoderBackRight;
    GamepadEx controller1;

    double L = 27, W=25;
    double R = Math.hypot(L/2, W/2); // length, width to be updated

    swerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;

    IMU imu;

    @Override
    public void runOpMode() {


        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");


        servoFrontRight = hardwareMap.get(CRServo.class, "servoFrontRight");
        servoFrontLeft = hardwareMap.get(CRServo.class, "servoFrontLeft");
        servoBackRight = hardwareMap.get(CRServo.class, "servoBackRight");
        servoBackLeft = hardwareMap.get(CRServo.class, "servoBackLeft");

        aencoderFrontRight = hardwareMap.get(AnalogInput.class, "encoderFrontRight");
        aencoderFrontLeft = hardwareMap.get(AnalogInput.class, "encoderFrontLeft");
        aencoderBackRight = hardwareMap.get(AnalogInput.class, "encoderBackRight");
        aencoderBackLeft = hardwareMap.get(AnalogInput.class, "encoderBackLeft");
        encoderFrontRight = new absoluteAnalogEncoder(aencoderFrontRight, 233, true);
        encoderFrontLeft = new absoluteAnalogEncoder(aencoderFrontLeft, 340, true);
        encoderBackLeft = new absoluteAnalogEncoder(aencoderBackLeft, 37, true);
        encoderBackRight = new absoluteAnalogEncoder(aencoderBackRight, 135, true);

        moduleFrontRight = new swerveModule(motorFrontRight, servoFrontRight, encoderFrontRight, telemetry);
        moduleFrontLeft = new swerveModule(motorFrontLeft, servoFrontLeft, encoderFrontLeft, telemetry);
        moduleBackLeft = new swerveModule(motorBackLeft, servoBackLeft, encoderBackLeft, telemetry);
        moduleBackRight = new swerveModule(motorBackRight, servoBackRight, encoderBackRight, telemetry);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        controller1 = new GamepadEx(gamepad1);

        waitForStart();
        while(opModeIsActive()) {

            if(gamepad1.options) {
                imu.resetYaw();
            }

            double FWD = -gamepad1.left_stick_y;
            double STR = gamepad1.left_stick_x;
            double RCW = gamepad1.right_stick_x;

//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double temp = FWD*Math.cos(botHeading) + STR*Math.sin(botHeading);
//            STR = -FWD*Math.sin(botHeading) + STR*Math.cos(botHeading);
//            FWD = temp;         
// *******UNCOMMENT FOR SUSSY BAKA FIELD CENTRIC*************

            double A = STR - RCW*(L/R);
            double B = STR + RCW*(L/R);
            double C = FWD - RCW*(W/R);
            double D = FWD + RCW*(W/R);

            double ws1 = Math.hypot(B, C);  double wa1 = Math.atan2(B, C)*180/Math.PI;
            double ws2 = Math.hypot(B, D); double wa2 = Math.atan2(B, D)*180/Math.PI;
            double ws3 = Math.hypot(A, D); double wa3 = Math.atan2(A, D)*180/Math.PI;
            double ws4 = Math.hypot(A, C); double wa4 = Math.atan2(A, C)*180/Math.PI;

            double max = ws1;
            if(ws2 > max) max = ws2; if(ws3 > max) max = ws3; if(ws4 > max) max = ws4;
            if(max > 1) { ws1/=max; ws2/=max; ws3/=max; ws4/=max; }

            telemetry.addData("OX1:", gamepad1.left_stick_x);
            telemetry.addData("OY1:", gamepad1.left_stick_y);
            telemetry.addData("ws1: ", ws1);
            telemetry.addData("wa1: ", wa1);

            telemetry.addData("ws4: ", ws4);

            telemetry.addData("ws4: ", ws4);
            telemetry.addData("wa4: ", wa4);

            moduleFrontRight.drive(ws1, wa1);
            moduleFrontLeft.drive(ws2, wa2);
            moduleBackLeft.drive(ws3, wa3);
            moduleBackRight.drive(ws4, wa4);

            telemetry.addData("encoderFrontLeft", aencoderFrontLeft.getVoltage() / 3.3*360);
            telemetry.addData("encoderFrontRight", aencoderFrontRight.getVoltage() / 3.3 * 360);
            telemetry.addData("encoderBackLeft", aencoderBackLeft.getVoltage()/3.3*360);
            telemetry.addData("encoderBackRight", aencoderBackRight.getVoltage() / 3.3 * 360);

            telemetry.update();

        }

    }

}