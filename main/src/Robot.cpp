#include <iostream>
#include <memory>
#include <string>
#include "WPILib.h"
#include "AHRS.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>

class Robot: public frc::IterativeRobot {
private:

	//Drive Motors
	VictorSP *m_leftDrive0; //4
	VictorSP *m_leftDrive1; //1
	VictorSP *m_rightDrive2; //2
	VictorSP *m_rightDrive3; //3

	//VictorSPSP *m_thing;


	CameraServer *USB;

	//Shooter Motors

	//Test Motors

	//Controllers
	Joystick *m_Joystick;
	XboxController *m_Gamepad;


	void RobotInit(void) override
	{
		m_leftDrive0 = new VictorSP(0);
		m_leftDrive1 = new VictorSP(1);
		m_rightDrive2 = new VictorSP(2);
		m_rightDrive3 = new VictorSP(3);

		//m_thing = new VictorSP(4);

		m_Gamepad = new XboxController(1);
		m_Joystick = new Joystick(0);

		USB = new CameraServer;
	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{
		USB->
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
		//operateThing();
		teleDrive();
	}

	void TestPeriodic() {

	}

	/*void operateThing() {
		if (m_Gamepad->GetYButton())
			m_thing->SetSpeed(1.0);
		else
			m_thing->SetSpeed(0.0);
	}*/

	void teleDrive() {
		float leftSpeed = limit(m_Joystick->GetY() - m_Joystick->GetX());
		float rightSpeed = limit(-m_Joystick->GetY() - m_Joystick->GetX());

		m_leftDrive0->SetSpeed(leftSpeed);
		m_leftDrive1->SetSpeed(leftSpeed);
		m_rightDrive2->SetSpeed(rightSpeed);
		m_rightDrive3->SetSpeed(rightSpeed);
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
