#include "SimModeCar.h"
#include "UObject/ConstructorHelpers.h"

//#include "vehicles/car/api/CarApiBase.hpp"
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "CarPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "vehicles/car/api/CarRpcLibServer.hpp"

extern CORE_API uint32 GFrameNumber;

void ASimModeCar::BeginPlay()
{
    Super::BeginPlay();

    initializePauseState();
}

void ASimModeCar::initializePauseState()
{
    pause_period_ = 0;
    pause_period_start_ = 0;
    pause(false);
}

void ASimModeCar::continueForTime(double seconds)
{
    pause_period_start_ = ClockFactory::get()->nowNanos();
    pause_period_ = seconds * current_clockspeed_;
    pause(false);
}

void ASimModeCar::continueForFrames(uint32_t frames)
{
    targetFrameNumber_ = GFrameNumber + frames;
    frame_countdown_enabled_ = true;
    pause(false);
}

FString ASimModeCar::GetMCMsg(const FString& vehicle_name)
{
    //FString to std::string
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    //auto car_api = car_sim_api->getPawnApi();
    auto msg = car_sim_api->getVehicleApi()->getCarMCMsg();
    //std::string to FString
    FString fMsg = FString(msg.msg.c_str());
    return fMsg;
}

void ASimModeCar::SendMCMsg(const FString& msg, const FString& vehicle_name)
{
    //FString to std::string
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));
    std::string smsg(TCHAR_TO_UTF8(*msg));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    auto car_api = car_sim_api->getVehicleApi();

    msr::airlib::CarApiBase::CarMCMsg MCMsg(
        smsg,
        car_sim_api->getVehicleApi()->clock()->nowNanos());
    car_sim_api->getVehicleApi()->sendCarMCMsg(MCMsg);
}

void ASimModeCar::FromUEGetString_Implementation(FString& outStr1, FString& outStr2, const FString& vehicle_name)
{
    //FString to std::string
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    if (car_sim_api != nullptr) {
        auto car_api = car_sim_api->getVehicleApi();
        if (car_api != nullptr) {
            auto data = car_sim_api->getVehicleApi()->getCarCustomString();
            //std::string to FString
            outStr1 = FString(data.str1.c_str());
            outStr2 = FString(data.str2.c_str());

            //if (outStr1 != "" && outStr2 != "")
            //	UKismetSystemLibrary::PrintString(this, "fromUECarCustomString: " + outStr1 + " : " + outStr2);
        }
    }
}

void ASimModeCar::ToUESetString_Implementation(const FString& str1, const FString& str2, const FString& vehicle_name)
{
    //UKismetSystemLibrary::PrintString(this, "ToUESetString: " + str1 + " : " + str2);
    //FString to std::string
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));
    std::string s1(TCHAR_TO_UTF8(*str1));
    std::string s2(TCHAR_TO_UTF8(*str2));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    auto car_api = car_sim_api->getVehicleApi();

    msr::airlib::CarApiBase::CustomStrData csd(
        s1,
        s2,
        car_sim_api->getVehicleApi()->clock()->nowNanos());
    car_sim_api->getVehicleApi()->setUECarCustomString(csd);
}

bool GetVehicleAttackFromAPI_Implementation(const FString& vehicle_name = "")
{
    //FString to std::string
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    if (car_sim_api != nullptr) {
        auto car_api = car_sim_api->getVehicleApi();
        if (car_api != nullptr) {
            auto data = car_api->getAttackState();
            return data.isAttack;
        }
    }
    return false;
}

void SetVehicleStateToAPI_Implementation(bool isAlive, const FString& vehicle_name = "")
{
    std::string vname(TCHAR_TO_UTF8(*vehicle_name));

    auto sim_api = getVehicleSimApi(vname);
    auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    if (car_sim_api != nullptr) {
        auto car_api = car_sim_api->getVehicleApi();
        if (car_api != nullptr) {
            msr::airlib::CarApiBase::VehicleState vs(
                isAlive,
                car_api->clock()->nowNanos());
            car_api->updateVehicleState(vs);
        }
    }
}

void ASimModeCar::setupClockSpeed()
{
    Super::setupClockSpeed();

    current_clockspeed_ = getSettings().clock_speed;

    //setup clock in PhysX
    UAirBlueprintLib::setUnrealClockSpeed(this, current_clockspeed_);
    UAirBlueprintLib::LogMessageString("Clock Speed: ", std::to_string(current_clockspeed_), LogDebugLevel::Informational);
}

void ASimModeCar::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    if (!isPaused())
        ClockFactory::get()->stepBy(DeltaSeconds);

    if (pause_period_start_ > 0) {
        if (ClockFactory::get()->elapsedSince(pause_period_start_) >= pause_period_) {
            if (!isPaused())
                pause(true);

            pause_period_start_ = 0;
        }
    }

    if (frame_countdown_enabled_) {
        if (targetFrameNumber_ <= GFrameNumber) {
            if (!isPaused())
                pause(true);

            frame_countdown_enabled_ = false;
        }
    }

    // if (bShowMCMsg) {
    // 	auto sim_api = getVehicleSimApi();
    // 	auto car_sim_api = static_cast<CarPawnSimApi*>(sim_api);
    // 	//auto car_api = car_sim_api->getPawnApi();
    // 	auto recvMsg = car_sim_api->getVehicleApi()->getRecvCarMCMsg();
    // 	auto sentMsg = car_sim_api->getVehicleApi()->getSentCarMCMsg();
    // 	//std::string to FString
    // 	recvMCMsg = FString(recvMsg.msg.c_str());
    // 	sentMCMsg = FString(sentMsg.msg.c_str());
    // }

    if (!bIsApiControlled) {
        auto vsim_api = getVehicleSimApi();
        auto vcar_sim_api = static_cast<CarPawnSimApi*>(vsim_api);
        if (vcar_sim_api->getVehicleApi()->isApiControlEnabled()) {
            TArray<AActor*> pawns;
            getExistingVehiclePawns(pawns);

            for (AActor* pawn : pawns) {
                auto vehicle_pawn = static_cast<ACarPawn*>(pawn);
                vehicle_pawn->ApiControlEvent.Broadcast();
            }
            bIsApiControlled = true;
        }
    }
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeCar::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::CarRpcLibServer(
        getApiProvider(), getSettings().api_server_address, getSettings().api_port));
#endif
}

void ASimModeCar::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeCar::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return ((vehicle_type == AirSimSettings::kVehicleTypePhysXCar) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduRover));
}

std::string ASimModeCar::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "")
        pawn_path = "DefaultCar";

    return pawn_path;
}

PawnEvents* ASimModeCar::getVehiclePawnEvents(APawn* pawn) const
{
    return static_cast<TVehiclePawn*>(pawn)->getPawnEvents();
}
const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeCar::getVehiclePawnCameras(
    APawn* pawn) const
{
    return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeCar::initializeVehiclePawn(APawn* pawn)
{
    auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn);
    vehicle_pawn->initializeForBeginPlay(getSettings().engine_sound);
}
std::unique_ptr<PawnSimApi> ASimModeCar::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    auto vehicle_pawn = static_cast<TVehiclePawn*>(pawn_sim_api_params.pawn);
    auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new CarPawnSimApi(pawn_sim_api_params,
                                                                         vehicle_pawn->getKeyBoardControls()));
    vehicle_sim_api->initialize();
    vehicle_sim_api->reset();
    return vehicle_sim_api;
}
msr::airlib::VehicleApiBase* ASimModeCar::getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                        const PawnSimApi* sim_api) const
{
    const auto car_sim_api = static_cast<const CarPawnSimApi*>(sim_api);
    return car_sim_api->getVehicleApi();
}
