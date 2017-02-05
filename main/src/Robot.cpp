#include <iostream>
#include <memory>
#include <string>
#include <ctime>
//#include <time.h>
#include <fstream>
#include "WPILib.h"
#include "CANTalon.h"
#include "SimPID.h"
#include "AHRS.h"
#include "VisionGearAlpha.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

//CONSTANTS
#define SHOOTER_RPM 4000
#define NATIVE_TO_RPM 0.146484375f
#define GP_R 6
#define GP_L 5
#define SHOOTER_RATIO setPoint/4096

class Robot: public frc::IterativeRobot {
private:
	//variables
	int manipState;
	int fileCount;
	int autoMode;
	int autoState;
	int turnSide;

	char buffer[50];
	std::ofstream file;

	//Victors
	VictorSP *m_leftDrive0;
	VictorSP *m_leftDrive1;
	VictorSP *m_rightDrive2;
	VictorSP *m_rightDrive3;

	VictorSP *m_intake;
	//VictorSP *m_shooter1;
	//VictorSP *m_agitator;
	VictorSP *m_elevate;
	VictorSP *m_intoShooter;

	//Talons
	CANTalon *m_shooter1;
	//CANTalon *//;

	//Encoders
	Encoder *m_LeftEncoder;
	Encoder *m_RightEncoder;

	//navx
	AHRS *nav;

	//Vision
	CameraServer *m_gearCam;
	CameraServer *m_shotCam;

	double last_turn;
	int aim_attempts;

	//Controllers
	Joystick *m_Joystick;
	XboxController *m_Gamepad;

	//pneumatics
	Solenoid *m_shiftHigh, *m_shiftLow;
	Solenoid *m_gearHoldOut, *m_gearHoldIn;

	//PIDS
	SimPID *speedToPowerPID;
	SimPID *drivePID;
	SimPID *turnPID;
	SimPID *visionPID;

	//LED
	Relay *m_gearLED;
	Relay *m_shotLED;

	//Time
	timeval tv;
	Timer *agTimer;

	void RobotInit(void) override
	{
		//variables
		manipState = 0;
		fileCount = 1;
		autoMode = 0;
		turnSide = 1;

		file.open("/home/lvuser/pid.csv", std::ios::out);

		//Victors
		m_leftDrive0 = new VictorSP(0);
		m_leftDrive1 = new VictorSP(1);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive3 = new VictorSP(3);
		//m_agitator = new VictorSP(4);
		m_intoShooter = new VictorSP(5);
		m_elevate = new VictorSP(7);
		m_intake = new VictorSP(9);

		//m_shooter1 = new VictorSP(8);

		//Talons
		m_shooter1 = new CANTalon(1);
		m_shooter1->SetControlMode(CANSpeedController::kSpeed);
		m_shooter1->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
		m_shooter1->ConfigEncoderCodesPerRev(4096);
		m_shooter1->SetSensorDirection(true);
		m_shooter1->SetPID(0.01, 0, 0.1, 0.028);
		m_shooter1->SetCloseLoopRampRate(0);
		m_shooter1->SetAllowableClosedLoopErr(0);
		m_shooter1->SelectProfileSlot(0);

		// = new CANTalon(2);
		//->SetControlMode(CANSpeedController::kSpeed);

	//	m_shooter12 = new CANTalon(1);

		//encoders
		m_LeftEncoder = new Encoder(0,1);
		m_RightEncoder = new Encoder(2,3);

		//navx
		nav = new AHRS(SPI::Port::kMXP);

		//vision
		last_turn = 0;
		aim_attempts = 0;

		//controllers
		m_Gamepad = new XboxController(1);
		m_Joystick = new Joystick(0);

		//pneumatics
		m_shiftHigh = new Solenoid(1);
		m_shiftLow = new Solenoid(2);
		m_gearHoldOut = new Solenoid(3);
		m_gearHoldIn = new Solenoid(4);

		//LED
		m_gearLED = new Relay(0);
		m_shotLED = new Relay(1);

		//PID
		speedToPowerPID = new SimPID;

		drivePID = new SimPID(0, 0, 0);
		drivePID->setMinDoneCycles(1);
		drivePID->setMaxOutput(0.7);

		turnPID = new SimPID(0, 0, 0);
		turnPID->setMinDoneCycles(1);
		turnPID->setMaxOutput(0.3);

		visionPID = new SimPID(0, 0, 0);
		visionPID->setMinDoneCycles(10);
		visionPID->setMaxOutput(0.4);

		//Time
		agTimer = new Timer();
		agTimer->Reset();
		agTimer->Stop();
	}

	void DisabledInit()
	{
		m_gearLED->Set(Relay::kOff);
		m_shotLED->Set(Relay::kOff);
	}

	void DisabledPeriodic()
	{
		DriverStation::ReportError("Left encoder" + std::to_string((long)m_LeftEncoder->Get()) + "Right Encoder" + std::to_string((long)m_RightEncoder->Get()) + "Gyro" + std::to_string(nav->GetYaw()));

		if(m_Joystick->GetRawButton(1)) {
			turnSide = 1;
			DriverStation::ReportError("Turn Side: RED");
		}
		else if(m_Joystick->GetRawButton(2)) {
			turnSide = -1;
			DriverStation::ReportError("Turn Side: BLUE");
		}
	}

	void AutonomousInit()
	{
		autoState = 0;
		m_gearLED->Set(Relay::kOn);
		m_shotLED->Set(Relay::kOn);
	}

	void AutonomousPeriodic()
	{
		switch(autoMode) {
		case 0: //still
			m_leftDrive0->SetSpeed(0.f);
			m_leftDrive1->SetSpeed(0.f);
			m_rightDrive2->SetSpeed(0.f);
			m_rightDrive3->SetSpeed(0.f);
			//m_agitator->SetSpeed(0.f);
			m_intake->SetSpeed(0.f);
			m_shooter1->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			break;
		case 1: //load on right peg
			switch(autoState) {
			case 0:
				autoState++;
				break;
			case 1:

				break;
			}
			break;
		}
	}

	void TeleopInit()
	{
		tv.tv_sec = 0;
		tv.tv_usec = 0;
		m_gearLED->Set(Relay::kOn);
	}

	void TeleopPeriodic()
	{
		operateIntake();
		teleDrive();
		//operateShooter();
		//trim();
		ShooterPID();
		operateShift();
		operateGear();
	}

	void TestPeriodic() {

	}

//=====================TELEOP FUNCTIONS=======================

	void operateIntake() {
		if (m_Gamepad->GetRawButton(GP_R)) {
			m_intake->SetSpeed(1.0);
			m_elevate->SetSpeed(-1.0);
		}
		else if(m_Gamepad->GetRawButton(GP_L)) {
			m_intake->SetSpeed(-1.0);
			m_elevate->SetSpeed(1.0);
		}
		else {
			m_intake->SetSpeed(0.f);
			m_elevate->SetSpeed(0.f);
		}
	}

	/*void operateShooter()
	{
		if (m_Gamepad->GetAButton())
			m_shooter1->SetSpeed(1.0);
		else
			m_shooter1->SetSpeed(0.0);
	}
*/
	void trim(){
		float motorOutput = 0.5*m_Joystick->GetRawAxis(3) + 0.5;


		m_shooter1->Set(motorOutput);
		float encoderRPM = m_shooter1->GetSpeed()*18.75f/128.f;

		DriverStation::ReportError("Encoder speed" + std::to_string((long)encoderRPM));


	}

	void ShooterPID() {

		//int setPoint = 1500 * (0.5 * m_Joystick->GetRawAxis(2) + 0.5) + 3000;
		int setPoint = 3800;
		gettimeofday(&tv, 0);

		float encoderRPM = m_shooter1->GetSpeed();
		DriverStation::ReportError("setpoint: "+ std::to_string(setPoint) + "Encoder speed" + std::to_string((long)encoderRPM));



		if(m_Gamepad->GetAButton()) {
			agTimer->Start();
			m_shooter1->SetSetpoint(setPoint);
			//->Set(SHOOTER_RATIO);

			if(agTimer->Get() > 1.0)
				m_intoShooter->SetSpeed(0.7);

			DriverStation::ReportError("speed error " + std::to_string(m_shooter1->GetClosedLoopError()*NATIVE_TO_RPM));

			sprintf(buffer, "%d:%d , %d , %d , %f\n", (int)tv.tv_sec, (int)tv.tv_usec, setPoint, (int)encoderRPM, m_shooter1->GetClosedLoopError()*NATIVE_TO_RPM);
			file << buffer;

			fileCount++;
		}
		else
		{
			m_shooter1->SetSetpoint(0);
			//->Set(0.f);
			m_intoShooter->SetSpeed(0.f);
			agTimer->Reset();
			agTimer->Stop();
		}
	}

	void teleDrive() {
		float leftSpeed = limit(m_Joystick->GetY() - m_Joystick->GetX());
		float rightSpeed = limit(-m_Joystick->GetY() - m_Joystick->GetX());

		m_leftDrive0->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}

/*	void manipulatorControl(void) {

		switch(manipState)
		{
		case 0:
			//m_agitator->SetSpeed(0);
			//m_shooter1->Set(0);
			////->Set(0);

			if(m_Gamepad->GetAButton())
			{
				manipState = 1;
			}

			break;

		case 1:
			//m_agitator->SetSpeed(0);
			//m_shooter1->Set(0);
			////->Set(0);

			if(m_Gamepad->GetBButton())
			{
				manipState = 2;
			}

			break;

		case 2:
			//m_agitator->SetSpeed(1.0);
			//m_shooter1->Set(1.0);
			////->Set(1.0);

			if(m_Gamepad->GetXButton())
			{
				manipState = 1;
			}

			break;
		}
	}*/

	void operateShift() {
		if(m_Joystick->GetRawButton(1)) {
			m_shiftHigh->Set(true);
			m_shiftLow->Set(false);
		}
		else {
			m_shiftHigh->Set(false);
			m_shiftLow->Set(true);
		}
	}

	void operateGear() {
		if(m_Gamepad->GetYButton()) {
			m_gearHoldIn->Set(true);
			m_gearHoldOut->Set(false);
		}
		else if(m_Gamepad->GetXButton()) {
			m_gearHoldIn->Set(false);
			m_gearHoldOut->Set(true);
		}
	}

//=====================VISION FUNCTIONS=====================

	bool aimAtTarget() {
		float turn = last_turn;

		return 0;
	}

//=====================AUTO FUNCTIONS=====================

	bool autoDrive(int distance, int angle) {
		int currentDist = (m_RightEncoder->Get() + m_LeftEncoder->Get()) / 2;
		int currentAngle = nav->GetYaw();

		drivePID->setDesiredValue(distance);
		turnPID->setDesiredValue(angle);

		float drive = -drivePID->calcPID(currentDist);
		float turn = -turnPID->calcPID(currentAngle);

		m_leftDrive0->SetSpeed(limit(drive + turn));
		m_leftDrive1->SetSpeed(limit(drive + turn));
		m_rightDrive2->SetSpeed(-limit(drive - turn));
		m_rightDrive3->SetSpeed(-limit(drive - turn));

		return drivePID->isDone() && turnPID->isDone();
	}

//=======================MATHY FUNCTIONS============================
	float limit(float s) {
		if (s > 1)
			return 1;
		else if (s < -1)
			return -1;
		return s;
	}

};

START_ROBOT_CLASS(Robot)
