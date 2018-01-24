#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <fstream>
#include <thread>
#include <list>
#include <mutex>
#include <sstream>
#include "WPILib.h"
#include "SimPID.h"
#include "AHRS.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include "shiftlib.h"
#include "ctre/Phoenix.h"

#define GP_L 5
#define GP_R 6

#define AUX_PWM

class Robot: public frc::IterativeRobot {
public:

	Joystick *m_Joystick; // The place where you define your variables
	Joystick *m_Joystick2;

	VictorSP *m_LFMotor; // ^
	VictorSP *m_LBMotor; // ^
	VictorSP *m_RFMotor; // ^
	VictorSP *m_RBMotor; // ^
	VictorSP *m_intakeL; // left intake wheel
	VictorSP *m_intakeR; // right intake wheel

	Solenoid *m_shiftHigh;
	Solenoid *m_shiftLow;
	int driveState;
	int autoState, autoMode;
	//Talon

#ifdef AUX_PWM
	VictorSP *m_elevatorPWM;
#else
	TalonSRX *m_elevatorCAN;
#endif

	XboxController *m_Gamepad;

	Solenoid *m_testExtend;
	Solenoid *m_testRetract;

	Encoder *m_leftEncoder;
	Encoder *m_rightEncoder;

	SimPID *m_drivePID;
	SimPID *m_turnPID;
	SimPID *m_finalTurnPID;

	Path *p_step_one;
	Path *p_step_two;
	Path *p_step_three;

	PathFollower *BBYCAKES;

	AHRS *nav;

	void RobotInit() {
		m_LFMotor = new VictorSP(1); // The place where you initialize your variables to their victors and which port on the RoboRio/computer
		m_LFMotor->SetSafetyEnabled(true);
		m_LBMotor = new VictorSP(0); // ^
		m_LBMotor->SetSafetyEnabled(true);
		m_RFMotor = new VictorSP(2); // ^
		m_RFMotor->SetSafetyEnabled(true);
		m_RBMotor = new VictorSP(3); // ^
		m_RBMotor->SetSafetyEnabled(true);

		m_intakeL = new VictorSP(4); // left intake
		m_intakeR = new VictorSP(5); // right intake

		m_Joystick = new Joystick(0); // ^
		m_Joystick2 = new Joystick(1);// ^

		m_Gamepad = new XboxController(2);

		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);
		m_testExtend = new Solenoid(2);
		m_testRetract = new Solenoid(3);

#ifdef AUX_PWM
		m_elevatorPWM = new VictorSP(6);
#else
		m_elevatorCAN = new TalonSRX(1);
		m_elevatorCAN->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
#endif

		nav = new AHRS(SPI::Port::kMXP);
		nav->Reset();

		driveState = 7;
		autoState = 0;

		m_leftEncoder = new Encoder(2,3);
		m_rightEncoder = new Encoder(0,1);

		m_turnPID = new SimPID(1.0, 0, 0.02, 0, 5.0);
		m_turnPID->setContinuousAngle(true);
		m_drivePID = new SimPID(0.001, 0, 0.002, 0, 200);
		m_drivePID->setMaxOutput(0.9);
		m_finalTurnPID = new SimPID(0.9, 0, 0.02, 0, 5.0);
		m_finalTurnPID->setContinuousAngle(true);

		BBYCAKES = new PathFollower(500, PI/3, m_drivePID, m_turnPID, m_finalTurnPID);
		BBYCAKES->setIsDegrees(true);

		int zero[2] = {0, 0};
		int step_one[2] = {-5000, 0};
		int step_two[2] = {0, -2887};
		int end[2] = {-5000, -2887};

		p_step_one = new PathLine(zero, step_one, 10);
		p_step_two = new PathLine(step_one, step_two, 10);
		p_step_three = new PathLine(step_two, end, 10);
	}

	void DisabledPeriodic() {
		long long int LRead = m_leftEncoder->Get();
		long long int RRead = m_rightEncoder->Get();
		long double GRead = nav->GetYaw();
		for(int i = 9; i <= 11; i++) {
			if(m_Joystick->GetRawButton(i))
				driveState = i;
		}
//		DriverStation::ReportError("Left Encoder: " + std::to_string(LRead) + " | Right Encoder: " + std::to_string(RRead));
		frc::SmartDashboard::PutNumber("Left Encoder", LRead);
		frc::SmartDashboard::PutNumber("Right Encoder", RRead);
		frc::SmartDashboard::PutNumber("Gyro", GRead);
		frc::SmartDashboard::PutNumber("Auto Mode", autoMode);
		frc::SmartDashboard::PutNumber("Drive Mode", driveState);
	}

	void AutonomousInit() override {
		m_LFMotor->SetSpeed(0.f);
		m_LBMotor->SetSpeed(0.f);
		m_RFMotor->SetSpeed(0.f);
		m_RBMotor->SetSpeed(0.f);
		m_leftEncoder->Reset();
		m_rightEncoder->Reset();
//		nav->Reset();
		autoState = 0;
	}

	void AutonomousPeriodic() {
	/*	switch(autoState) {
		case 0:
			BBYCAKES->initPath(p_step_one, PathForward, -30);
			autoState++;
			break;
		case 1:
			BBYCAKES->initPath(p_step_two, PathBackward, 0);
			if(advancedAutoDrive())
				autoState++;
			break;
		case 2:
			BBYCAKES->initPath(p_step_three, PathForward, 360);
			if(advancedAutoDrive())
				autoState++;
			break;
		case 3:
			advancedAutoDrive();
			break;
		}*/

	}

	void TeleopInit() {
		m_LFMotor->SetSpeed(0.f);
		m_LBMotor->SetSpeed(0.f);
		m_RFMotor->SetSpeed(0.f);
		m_RBMotor->SetSpeed(0.f);
		m_intakeL->SetSpeed(0.f);
		m_intakeR->SetSpeed(0.f);
	}

	void applesAreTheBest() {
		if(m_Joystick->GetRawButton(1)) {
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else {
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	void simpleElev() {
		float elevSpeed = 0.65*limit(m_Gamepad->GetRawAxis(1));
#ifdef AUX_PWM
		m_elevatorPWM->Set(limit(-elevSpeed));
#else
		m_elevatorCAN->Set(ControlMode::PercentOutput, limit(-elevSpeed + 0.1));
#endif
	}

	void advancedElev() {

	}

	void TeleopPeriodic() {
		applesAreTheBest();
		simpleElev();
		switch(driveState) {
		case 7:
			arcadeDrive();
			break;
		case 8:
			tankDrive();
			break;
		case 9:
			cheezyDrive();
			break;
		}
		simpleIntake();
	}

/*	bool advancedAutoDrive() {
		float leftSpeed, rightSpeed;
		if(BBYCAKES->followPathByEnc(m_leftEncoder->Get(), m_rightEncoder->Get(), nav->GetYaw(), leftSpeed, rightSpeed) == 0){
			m_LFMotor->SetSpeed(leftSpeed);
			m_LBMotor->SetSpeed(leftSpeed);
			m_RFMotor->SetSpeed(rightSpeed);
			m_RBMotor->SetSpeed(rightSpeed);
		}
		printf("path follow left: %f, right: %f\n", leftSpeed, rightSpeed);
		return BBYCAKES->isDone();
	}*/

	void arcadeDrive() {
		//R = +y - x;
		//L = -y - x;
		float joyX;
		float joyY;

		joyX = -limit(expo(m_Joystick->GetX(), 3)); // Getting the X position from the joystick
		joyY = -limit(expo(m_Joystick->GetY(), 2)); // Getting the Y position from the joystick

		m_LFMotor->SetSpeed(-joyY + joyX);
		m_LBMotor->SetSpeed(-joyY + joyX);
		m_RFMotor->SetSpeed(joyY + joyX);
		m_RBMotor->SetSpeed(joyY + joyX);
	}

	void tankDrive() {
		float joy1Y;
		float joy2Y;

		joy1Y = -limit(expo(m_Joystick->GetY(), 2));
		joy2Y = -limit(expo(m_Joystick2->GetY(), 2));

		m_LFMotor->SetSpeed(-joy1Y);
		m_LBMotor->SetSpeed(-joy1Y);
		m_RFMotor->SetSpeed(joy2Y);
		m_RBMotor->SetSpeed(joy2Y);
	}

	void simpleIntake() {
		if(!m_Gamepad->GetAButton()) {
			m_testExtend->Set(true);
			m_testRetract->Set(false);
		}
		else {
			m_testExtend->Set(false);
			m_testRetract->Set(true);
		}

		if(m_Gamepad->GetRawButton(GP_R)) {
			m_intakeL->SetSpeed(1.f);
			m_intakeR->SetSpeed(-1.f);
		}
		else if(m_Gamepad->GetRawButton(GP_L)) {
			m_intakeL->SetSpeed(-1.f);
			m_intakeR->SetSpeed(1.f);
		}
		else {
			m_intakeL->SetSpeed(0.f);
			m_intakeR->SetSpeed(0.f);
		}
	}

	void cheezyDrive() {
		float joy1Y;
		float joy2X;

		joy1Y = -limit(expo(m_Joystick->GetY(), 2));
		joy2X = -limit(expo(m_Joystick2->GetX(), 3));

		m_LFMotor->SetSpeed(-joy1Y - joy2X);
		m_LBMotor->SetSpeed(-joy1Y - joy2X);
		m_RFMotor->SetSpeed(joy1Y - joy2X);
		m_RBMotor->SetSpeed(joy1Y - joy2X);

	}

// #.f is used to tell the computer that the number here is not a whole number(1).
	float limit(float n) { // The range and domain are equal to 1.f and -1.f
		if(n>1.f)
			return 1.f; // If "n" is greater then 1.f then return positive 1.f
		if(n<-1.f)
			return -1.f; // If "n" is less then -1.f then return negative -1.f
		return n;
	}

	float expo(float b, int x) {
		float r = 1;
		if(x % 2 == 0) {
			for(int i = 1; i <= x; i++)
				r *= b;
			if(b >= 0)
				return r;
			else
				return -r;
		}
		else {
			for(int i = 1; i <= x; i++)
				r *= b;
			return r;
		}
	}

//	template <typename T>
/*	std::string to_string(const T& value) {
	    std::stringstream ss;
	    ss << value;
	    return ss.str();
	}*/

	void TestPeriodic() {

	}

private:

};

START_ROBOT_CLASS(Robot)
