#pragma once

#include "vehicles/car/api/CarApiBase.hpp"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "physics/Kinematics.hpp"
#include "CarPawn.h"

class CarPawnApi
{
public:
	typedef msr::airlib::ImageCaptureBase ImageCaptureBase;

	CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
		msr::airlib::CarApiBase* vehicle_api);

	void updateMovement(const msr::airlib::CarApiBase::CarControls& controls);

	msr::airlib::CarApiBase::CarState getCarState() const;

	void reset();
	void update();

	virtual ~CarPawnApi();

	//test
	UChaosVehicleMovementComponent* CVMC;
	bool fly;
	bool moveForwad;
	float elseValue;
	bool torqueState;
	float hight;
	bool moveControl;
	bool rightControl;
	bool rightControl_2;
	bool rotateControl;
	float rotateT;

	bool stopOne;
	bool stopOne2;

	int type;

	bool inputOff;
	//
	//PID move
	float Kp, Ki, Kd, error, last_error, integral;
	float OldInSpeed;
	float OldBrake;
	float localBrake;
	//PID Rotator

	float KpR, KiR, KdR, errorR, last_errorR, integralR;
	float OldInSpeedR;
	float OldRoataor;
	int OldInR;

	//

private:
	UChaosWheeledVehicleMovementComponent* movement_;
	msr::airlib::CarApiBase::CarControls last_controls_;
	ACarPawn* pawn_;
	const msr::airlib::Kinematics::State* pawn_kinematics_;
	msr::airlib::CarApiBase* vehicle_api_;

	// msr::airlib::CarApiBase::CarMCMsg recvMCMsg_;
	// msr::airlib::CarApiBase::CarMCMsg sentMCMsg_;

	//PID
	void PIDIni(float InKp, float InKi, float InKd);
	float calculate_control(float desired_state, float current_state, float dt);

	void PIDIniR(float InKp, float InKi, float InKd);
	float calculate_controlR(float desired_state, float current_state, float dt);
	float GetRotator(int speed);
};
