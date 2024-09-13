#include "CarPawnApi.h"
#include "AirBlueprintLib.h"

#include "ChaosVehicleManager.h"

CarPawnApi::CarPawnApi(ACarPawn* pawn, const msr::airlib::Kinematics::State* pawn_kinematics,
	msr::airlib::CarApiBase* vehicle_api)
	: pawn_(pawn), pawn_kinematics_(pawn_kinematics), vehicle_api_(vehicle_api)
{
	movement_ = CastChecked<UChaosWheeledVehicleMovementComponent>(pawn->GetVehicleMovement());

	CVMC = pawn->GetVehicleMovement();
	type = pawn->Type;

	elseValue = pawn->RotateSpeed;
	fly = false;
	rightControl = true;
	rightControl_2 = true;
	stopOne = false;
	stopOne2 = false;
	rotateControl = false;
	rotateT = 0.0f;

	inputOff = pawn->inputOff;

	PIDIni(pawn->InKp, pawn->InKi, pawn->InKd);
	PIDIniR(pawn_->InKpR, pawn_->InKiR, pawn_->InKdR);
	OldRoataor = pawn_->GetActorRotation().Yaw;
	OldBrake = 0.0f;
}

void CarPawnApi::updateMovement(const msr::airlib::CarApiBase::CarControls& controls)
{
	if (inputOff)
		return;


	last_controls_ = controls;

	if (!controls.is_manual_gear && movement_->GetTargetGear() < 0)
		movement_->SetTargetGear(0, true); //in auto gear we must have gear >= 0
	if (controls.is_manual_gear && movement_->GetTargetGear() != controls.manual_gear)
		movement_->SetTargetGear(controls.manual_gear, controls.gear_immediate);







	float localF = controls.throttle * 100.0f;

	float localRL = FMath::RadiansToDegrees(controls.steering);
	pawn_->MoveInputDelegate.Broadcast(localF, localRL);


	if (FMath::Abs(localRL) < 1.0f)
		localRL = 0;

	float inBrake = controls.brake;

	if (FMath::Abs(movement_->GetForwardSpeed()) <= 0.3f)
		localBrake = 0.0f;
	else
		localBrake = inBrake;

	UKismetSystemLibrary::PrintString(CVMC, "InputSpeed:" + FString::FromInt(localF), true, false, FLinearColor::Green, 0.0f);


	//解决键盘move 数值输入无效
	if (pawn_->MoveInputV != 0)
	{
		localF = pawn_->TestMoveSpeed * (FMath::Sign(controls.throttle));
	}

	UKismetSystemLibrary::PrintString(CVMC, "SetSpeed:" + FString::FromInt(localF), true, false, FLinearColor::Green, 0.0f);
	UKismetSystemLibrary::PrintString(CVMC, "CurrentMoveSpeed:" + FString::FromInt(movement_->GetForwardSpeed()), true, false, FLinearColor::Green, 0.0f);
	UKismetSystemLibrary::PrintString(CVMC, "InputRoator:" + FString::FromInt(controls.steering), true, false, FLinearColor::Green, 0.0f);


	///// 测试的功能

	if (localF < 0)
	{
		pawn_->onReversePressed();
		////!!!!!!!!!!!!!!!!!
	//	movement_->SetTargetGear(-1.0f, true);

		if (movement_->GetForwardSpeed() == 0.0f)
		{
			localRL = 0.1f;
			localF = 0.0f;
		}



	}
	else
		pawn_->onReverseReleased();

	///////////////////////////////////////////////////////////////////////////////////////////////////
	//TestPID !!速度不能超过最大值，变速后会导致移动瞬间加速卡顿
	//speed
	if (localF != OldInSpeed)
	{
		//减速刹车（仅前进）
		if (movement_->GetForwardSpeed() > 0.0f && movement_->GetForwardSpeed() > (localF + (localF > 0 ? 1.0f : -1.0f) * 5.0f))
		{
			localBrake = 1.0f;
			localF = 0.0f;
		}/////////////////////

		if (FMath::Abs(localF - OldInSpeed) > 10.0f) // 测试初始化 速度相差大于
			PIDIni(pawn_->InKp, pawn_->InKi, pawn_->InKd);

		OldInSpeed = localF;
	}
	if (FMath::Abs(localF) > 0.0f)
		localF = calculate_control(localF, movement_->GetForwardSpeed(), CVMC->GetWorld()->GetDeltaSeconds());




	//Rotator
	//if (localRL != OldInSpeedR)
	//{
	//	OldInSpeedR = localRL;
	//	PIDIniR(pawn_->InKpR, pawn_->InKiR, pawn_->InKdR);
	//}
	float localR = GetRotator(FMath::Sign(localRL));
	if (FMath::Abs(localRL) > 0.0f)
	{
		localBrake = 0.0f;
		localRL = calculate_controlR(localRL, localR, CVMC->GetWorld()->GetDeltaSeconds());
	}

	UKismetSystemLibrary::PrintString(CVMC, "CurrentRotatorSpeed:" + FString::FromInt(localR), true, false, FLinearColor::Yellow, 0.0f);
	UKismetSystemLibrary::PrintString(CVMC, "SetRotator:" + FString::FromInt(localRL), true, false, FLinearColor::Yellow, 0.0f);



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 刹车测试
	if (localBrake == 1)
	{//Test!!!!!!!!!!!!!
		PIDIni(pawn_->InKp, pawn_->InKi, pawn_->InKd);
		PIDIniR(pawn_->InKpR, pawn_->InKiR, pawn_->InKdR);
		//
		movement_->SetSteeringInput(controls.steering);

		int index = movement_->GetNumWheels();

		CVMC->SetYawInput(0.0f);
		for (int i = 0; i < index; i++)
		{
			movement_->SetTorqueCombineMethod(ETorqueCombineMethod::None, i);
		}
		CVMC->SetThrottleInput(0.0f);
		CVMC->SetHandbrakeInput(true);
		CVMC->SetBrakeInput(1.0f);

		stopOne = true;
		return;
	}
	else if (stopOne)
	{
		stopOne = false;
		CVMC->SetThrottleInput(1.0f);
		CVMC->SetBrakeInput(0.0f);
		CVMC->SetHandbrakeInput(false);
		stopOne2 = true;
	}
	else if (stopOne2)
	{
		stopOne2 = false;
		CVMC->SetThrottleInput(0.0f);
	}
	//

	switch (type)
	{
	case 0:

		moveForwad = (localF != 0.0f);

		if (moveForwad)
		{
			moveControl = true;

			if (controls.throttle > 0.0f)
			{
				CVMC->SetThrottleInput(localF);
				CVMC->SetBrakeInput(0.0f);
			}
			else
			{
				CVMC->SetBrakeInput(-1.0f * localF);
				CVMC->SetThrottleInput(0.0f);
			}
		}
		else
		{
			if (moveControl)
			{
				moveControl = false;
				CVMC->SetBrakeInput(0.0f);
				CVMC->SetThrottleInput(0.0f);
			}
		}


		//movement_->SetThrottleInput(controls.throttle);
		movement_->SetSteeringInput(localRL);

		//movement_->SetBrakeInput(controls.brake);
		movement_->SetHandbrakeInput(controls.handbrake);

		break;
	case 1:
		//movement_->SetBrakeInput(controls.brake);
		movement_->SetHandbrakeInput(controls.handbrake);


		//float localRL = controls.steering;
		//float R = controls.brake;
		//bool Sp = controls.handbrake;

		moveForwad = (localF != 0.0f);

		if (moveForwad)
		{
			moveControl = true;

			if (controls.throttle > 0.0f)
			{
				CVMC->SetThrottleInput(localF);
				CVMC->SetBrakeInput(0.0f);
			}
			else
			{
				CVMC->SetBrakeInput(-1.0f * localF);
				CVMC->SetThrottleInput(0.0f);
			}
		}
		else
		{
			if (moveControl)
			{
				moveControl = false;
				CVMC->SetBrakeInput(0.0f);
				CVMC->SetThrottleInput(0.0f);
			}
		}

		if (rotateControl)
		{
			rotateT -= CVMC->GetWorld()->GetDeltaSeconds();

			if (rotateT <= 0.0f)
			{
				rotateControl = false;
				movement_->Mass = 250.0f;
			}
		}


		if (localRL == 0.0f)
		{
			if (rightControl)
			{
				int index = movement_->GetNumWheels();

				rightControl = false;
				CVMC->SetYawInput(0.0f);

				CVMC->SetThrottleInput(0.0f);
				CVMC->SetHandbrakeInput(false);

				rotateControl = true;


				movement_->Mass = 4000.0f;
				rotateT = 0.6f;


				for (int i = 0; i < index; i++)
				{
					movement_->SetTorqueCombineMethod(ETorqueCombineMethod::None, i);
				}
			}
		}
		else
		{
			if (fly)
			{
				int index = movement_->GetNumWheels();

				CVMC->SetYawInput(0.0f);
				for (int i = 0; i < index; i++)
				{
					movement_->SetTorqueCombineMethod(ETorqueCombineMethod::None, i);
				}
				CVMC->SetThrottleInput(0.0f);
				CVMC->SetHandbrakeInput(false);
			}
			else
			{
				int index = movement_->GetNumWheels();

				float localYaw = localRL * FMath::Clamp(
					((FMath::Abs(movement_->GetForwardSpeed())) * 0.00001f),
					0.2f,
					1.0f)
					* elseValue;

				//			UKismetSystemLibrary::PrintString(CVMC, FString::SanitizeFloat(((FMath::Abs(movement_->GetForwardSpeed())) * 0.0001f)), true, false, FLinearColor::Blue, 0.0f);

				CVMC->SetYawInput(localYaw);
				rightControl = true;
				if (moveForwad)
				{
					if (rightControl_2)
					{
						rightControl_2 = false;
						for (int i = 0; i < index; i++)
						{
							movement_->SetTorqueCombineMethod(ETorqueCombineMethod::None, i);
						}
						CVMC->SetThrottleInput(0.0f);
						CVMC->SetHandbrakeInput(false);
					}
				}
				else
				{
					for (int i = 0; i < index; i++)
					{
						movement_->SetTorqueCombineMethod(ETorqueCombineMethod::Override, i);
					}
					CVMC->SetThrottleInput(1.0f);
					CVMC->SetHandbrakeInput(true);
					rightControl_2 = true;
				}
			}
		}


		break;

	}




	//movement_->SetThrottleInput(controls.throttle);
	//movement_->SetSteeringInput(controls.steering);
	//movement_->SetBrakeInput(controls.brake);
	//movement_->SetHandbrakeInput(controls.handbrake);
	movement_->SetUseAutomaticGears(!controls.is_manual_gear);
}

msr::airlib::CarApiBase::CarState CarPawnApi::getCarState() const
{
	msr::airlib::CarApiBase::CarState state(
		movement_->GetForwardSpeed() / 100, //cm/s -> m/s
		movement_->GetCurrentGear(),
		movement_->GetEngineRotationSpeed(),
		movement_->GetEngineMaxRotationSpeed(),
		last_controls_.handbrake,
		*pawn_kinematics_,
		vehicle_api_->clock()->nowNanos());
	return state;
}

void CarPawnApi::reset()
{
	vehicle_api_->reset();

	last_controls_ = msr::airlib::CarApiBase::CarControls();
	auto phys_comps = UAirBlueprintLib::getPhysicsComponents(pawn_);
	UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
		for (auto* phys_comp : phys_comps) {
			phys_comp->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
			phys_comp->SetPhysicsLinearVelocity(FVector::ZeroVector);
			phys_comp->SetSimulatePhysics(false);
		}
		movement_->ResetMoveState();
		movement_->SetActive(false);
		movement_->SetActive(true, true);
		vehicle_api_->setCarControls(msr::airlib::CarApiBase::CarControls());
		updateMovement(msr::airlib::CarApiBase::CarControls());
		movement_->ResetVehicleState();
		//auto pv = movement_->PVehicle;
		//if (pv) {
		//    pv->mWheelsDynData.setToRestState();
		//}
		//auto pvd = movement_->PVehicleDrive;
		//if (pvd) {
		//    pvd->mDriveDynData.setToRestState();
		//}
		},
		true);

	UAirBlueprintLib::RunCommandOnGameThread([this, &phys_comps]() {
		for (auto* phys_comp : phys_comps)
			phys_comp->SetSimulatePhysics(true);
		},
		true);
}

void CarPawnApi::update()
{
	vehicle_api_->updateCarState(getCarState());
	vehicle_api_->update();
}




// msr::airlib::CarApiBase::CarMCMsg CarPawnApi::getCarMCMsg() const
// {
//     // msr::airlib::CarApiBase::CarMCMsg msg(
//     //     sentMCMsg_,
//     //     vehicle_api_->clock()->nowNanos());

//     // UE_LOG(LogTemp, Log, TEXT("send MC Msg: %s"), *msg.msg.c_str());
//     // return msg;
//        return vehicle_api_->getCarMCMsg();
// }

// void CarPawnApi::setCarMCMsg(const msr::airlib::CarApiBase::CarMCMsg& msg)
// {
//     recvMCMsg_ = msg;
//     UE_LOG(LogTemp, Log, TEXT("receive MC Msg: %s"), *msg.msg.c_str());
// }

CarPawnApi::~CarPawnApi() = default;

void CarPawnApi::PIDIni(float InKp, float InKi, float InKd)
{
	Kp = InKp;
	Ki = InKi;
	Kd = InKd;
	error = 0;
	last_error = 0;
	integral = 0;
}

float CarPawnApi::calculate_control(float desired_state, float current_state, float dt)
{
	error = desired_state - current_state;

	// 步骤 4: 比例控制
	float P = Kp * error;

	// 步骤 5: 积分控制
	integral += error * dt;
	float	I = Ki * integral;

	// 步骤 6: 微分控制
	float derivative = (error - last_error) / dt;
	float	D = Kd * derivative;

	// 步骤 7: 计算控制信号
	float	control = P + I + D;

	// 步骤 8: 更新上一次的误差
	last_error = error;

	return control;

}

void CarPawnApi::PIDIniR(float InKp, float InKi, float InKd)
{
	KpR = InKp;
	KiR = InKi;
	KdR = InKd;
	errorR = 0;
	last_errorR = 0;
	integralR = 0;
}

float CarPawnApi::calculate_controlR(float desired_state, float current_state, float dt)
{
	errorR = desired_state - current_state;

	// 步骤 4: 比例控制
	float P = KpR * errorR;

	// 步骤 5: 积分控制
	integralR += errorR * dt;
	float	I = KiR * integralR;

	// 步骤 6: 微分控制
	float derivative = (errorR - last_errorR) / dt;
	float	D = KdR * derivative;

	// 步骤 7: 计算控制信号
	float	control = P + I + D;

	// 步骤 8: 更新上一次的误差
	last_errorR = errorR;

	return control;
}

float CarPawnApi::GetRotator(int speed)
{

	//因惯性存在问题
	float localAngle = pawn_->GetActorRotation().Yaw + 180.0f;
	float localMax = FMath::Max(localAngle, OldRoataor);
	float localMin = FMath::Min(localAngle, OldRoataor);
	float localValue1 = 360.0f - localMax + localMin;
	float localValue2 = localMax - localMin;


	bool localA = (speed > 0 && localAngle < OldRoataor);
	bool localB = (speed<0 && localAngle>OldRoataor);
	float localValue3 = (localA || localB) ? localValue1 : localValue2;
	float localValue4 = 0.0f;
	if (localValue3 < 300.0f)
		localValue4 = (localValue3 * FMath::Sign(speed)) / CVMC->GetWorld()->GetDeltaSeconds();
	else
		localValue4 = FMath::Sign(speed) * 1;

	if (speed == OldInR)
	{
		OldRoataor = localAngle;

		return localValue4;
	}
	else
	{
		if ((OldInR > 0.0f) ? (localAngle <= OldRoataor) : (localAngle >= OldRoataor))
		{
			OldInR = speed;
			OldRoataor = localAngle;
			return localValue4;
		}
		OldInR = speed;
	}

	return FMath::Sign(speed) * 1;

}
