#include <iostream>
#include <memory>
#include <string>
#include "WPILib.h"
#include "AHRS.h"
#include "CANTalon.h"
#include "SimPID.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

//CONSTANTS
#define SHOOTER_RPM

class Robot: public frc::IterativeRobot {
private:
	//variables
	int manipState;

	//Victors
	VictorSP *m_leftDrive0;
	VictorSP *m_leftDrive1;
	VictorSP *m_rightDrive2;
	VictorSP *m_rightDrive3;

	VictorSP *m_thing;
	//VictorSP *m_shooter;
	VictorSP *m_agitator;
	VictorSP *m_intakeWheel;

	//Talons
	CANTalon *m_shooter;
	//CANTalon *m_shooter2;

	//Encoders
	Encoder *m_LeftEncoder;
	Encoder *m_RightEncoder;

	//navx

	//Vision
	CameraServer *USB;

	//Controllers
	Joystick *m_Joystick;
	XboxController *m_Gamepad;

	//pneumatics
	Solenoid *m_shiftHigh, *m_shiftLow;

	//PIDS
	SimPID *speedToPowerPID;


	void RobotInit(void) override
	{
		//variables
		manipState = 0;

		//Victors
		m_leftDrive0 = new VictorSP(0);
		m_leftDrive1 = new VictorSP(1);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive3 = new VictorSP(3);

		m_agitator = new VictorSP(4);
		m_intakeWheel = new VictorSP(5);
		m_thing = new VictorSP(9);

		//m_shooter = new VictorSP(8);

		//Talons
		m_shooter = new CANTalon(0);
	//	m_shooter2 = new CANTalon(1);

		//encoders
		m_LeftEncoder = new Encoder(0,1);
		m_RightEncoder = new Encoder(2,3);\

		//navx

		//vision

		//controllers
		m_Gamepad = new XboxController(1);
		m_Joystick = new Joystick(0);

		//pneumatics
		m_shiftHigh = new Solenoid(0);
		m_shiftLow = new Solenoid(1);

		//PID
		speedToPowerPID = new SimPID;


	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{
		DriverStation::ReportError("Left encoder" + std::to_string((long)m_LeftEncoder->Get()) + "Right Encoder" + std::to_string((long)m_RightEncoder->Get()));
	}

	void AutonomousInit() override
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		operateIntake();
		teleDrive();
		//operateShooter();
		trim();
	}

	void TestPeriodic() {

	}

	void operateIntake() {
		if (m_Gamepad->GetYButton())
			m_thing->SetSpeed(1.0);
		else if(m_Gamepad->GetXButton())
			m_thing->SetSpeed(-1.0);
		else
			m_thing->SetSpeed(0.0);
	}

	/*void operateShooter()
	{
		if (m_Gamepad->GetAButton())
			m_shooter->SetSpeed(1.0);
		else
			m_shooter->SetSpeed(0.0);
	}
*/
	void trim(){
		float motorOutput = m_Joystick->GetRawAxis(3);

		m_shooter->Set(motorOutput);
		float encoderRPM = m_shooter->GetSpeed()*1875.f/128.f;

		DriverStation::ReportError("Encoder speed" + std::to_string((long)encoderRPM));


	}

	void teleDrive() {
		float leftSpeed = limit(m_Joystick->GetY() - m_Joystick->GetX());
		float rightSpeed = limit(-m_Joystick->GetY() - m_Joystick->GetX());

		m_leftDrive0->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
	}

	void manipulatorControl(void) {

		switch(manipState)
		{
		case 0:
			m_agitator->SetSpeed(0);
			m_intakeWheel->SetSpeed(0);
			//m_shooter1->Set(0);
			//m_shooter2->Set(0);

			if(m_Gamepad->GetAButton())
			{
				manipState = 1;
			}

			break;

		case 1:
			m_agitator->SetSpeed(0);
			m_intakeWheel->SetSpeed(1.0);
			//m_shooter1->Set(0);
			//m_shooter2->Set(0);

			if(m_Gamepad->GetBButton())
			{
				manipState = 2;
			}

			break;

		case 2:
			m_agitator->SetSpeed(1.0);
			m_intakeWheel->SetSpeed(0);
			//m_shooter1->Set(1.0);
			//m_shooter2->Set(1.0);

			if(m_Gamepad->GetXButton())
			{
				manipState = 1;
			}

			break;
		}

	}


	//bool speedToPower(float )

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
