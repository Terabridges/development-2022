package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files

import org.json.JSONException;
import org.json.JSONObject;
import org.json.JSONArray;

@Autonomous(name="RedOdometer", group="Pushbot")


public class AutonomousRedOdometer extends LinearOpMode {
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor arm = null;
    private Servo hand = null;
    private DcMotor wheel = null;
    private ElapsedTime runtime = new ElapsedTime();
    public void setPowers(double direction, double speed, double rotation){
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;


        double forward = Math.cos(direction)*speed;
        double side  =  Math.sin(direction)*speed;

        rightBackPower    = Range.clip((forward + side + rotation), -1.0, 1.0);
        leftBackPower   = Range.clip((forward - side - rotation), -1.0, 1.0);
        rightFrontPower    = Range.clip((forward - side + rotation), -1.0, 1.0);
        leftFrontPower   = Range.clip((forward + side - rotation), -1.0, 1.0);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }
    private void waitOneSecond() {
        sleep(1000);
    }
    @Override
    public void runOpMode() {

        String data="data.json";
        try {
            File myObj = new File("data.json");
            Scanner myReader = new Scanner(myObj);
            while (myReader.hasNextLine()) {
                data = data + myReader.nextLine();
            }
            myReader.close();
        } catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }

        // parse the string as a json object
        String text = null;
        try {
            // pick this
            JSONObject aInstruction = new JSONObject(data);
            text = aInstruction.get("Text").toString();
            // or that
            //JSONArray manyInstructions = new JSONArray(data);
        } catch (JSONException e) {
            e.printStackTrace();
        }

//        ArrayList<double[]> Instructions = new ArrayList<double[]>();
//        double instruction1[] = {1, 2, 3};
//        Instructions.add(instruction1);
        leftBack  = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        arm = hardwareMap.get(DcMotor.class, "arm");
        hand = hardwareMap.get(Servo.class, "hand");
        wheel = hardwareMap.get(DcMotor.class, "wheel");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        wheel.setDirection(DcMotor.Direction.FORWARD);
        runtime.reset();
        waitForStart();

        double startTime = getRuntime();

        double xAmt = (Math.random() - 0.5)/2;
        double yAmt = (Math.random() - 0.5)/2;
        while(true) {

            double time = getRuntime() + 1;
            while(getRuntime() < time){
                double xPos = leftFront.getCurrentPosition() + rightBack.getCurrentPosition() - rightFront.getCurrentPosition() - leftBack.getCurrentPosition();
                double yPos = leftFront.getCurrentPosition() + rightBack.getCurrentPosition() + rightFront.getCurrentPosition() + leftBack.getCurrentPosition();
                if(xPos > 1000){
                    xAmt = Math.abs(xAmt);
                }
                if(xPos < -1000){
                    xAmt = -Math.abs(xAmt);
                }
                if(yPos > 1000){
                    yAmt = Math.abs(yAmt);
                }
                if(yPos < -1000){
                    yAmt = -Math.abs(yAmt);
                }
                leftFront.setPower(yAmt + xAmt);
                rightFront.setPower(yAmt - xAmt);
                leftBack.setPower(yAmt - xAmt);
                rightBack.setPower(yAmt + xAmt);

                telemetry.clear();
                telemetry.addData("TEXT", text);
                telemetry.addData("time", time - getRuntime());
                telemetry.addData("x", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() - rightFront.getCurrentPosition() - leftBack.getCurrentPosition());
                telemetry.addData("y", leftFront.getCurrentPosition() + rightBack.getCurrentPosition() + rightFront.getCurrentPosition() + leftBack.getCurrentPosition());
                telemetry.update();

            }

            waitOneSecond();
        }

//        stop();
    }

}