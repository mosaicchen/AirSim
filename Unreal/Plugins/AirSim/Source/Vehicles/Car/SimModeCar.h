#pragma once

#include "CoreMinimal.h"

#include "SimMode/SimModeBase.h"
#include "CarPawn.h"
#include "common/Common.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "SimModeCar.generated.h"

UCLASS()
class AIRSIM_API ASimModeCar : public ASimModeBase
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;

    virtual void continueForTime(double seconds) override;
    virtual void continueForFrames(uint32_t frames) override;

    // UFUNCTION(BlueprintCallable)
    // FString GetMCMsg(const FString& vehicle_name = "");

    // UFUNCTION(BlueprintCallable)
    // void SendMCMsg(const FString& msg, const FString& vehicle_name = "");

    // UPROPERTY(BlueprintReadWrite, EditAnywhere)
    // bool bShowMCMsg = false;
    // UPROPERTY(BlueprintReadWrite, EditAnywhere)
    // FString recvMCMsg;
    // UPROPERTY(BlueprintReadWrite, EditAnywhere)
    // FString sentMCMsg;

    UFUNCTION(BlueprintNativeEvent, BlueprintCallable)
    void FromUEGetString(FString& outStr1, FString& outStr2, const FString& vehicle_name = "");
    void FromUEGetString_Implementation(FString& outStr1, FString& outStr2, const FString& vehicle_name = "");

    UFUNCTION(BlueprintNativeEvent, BlueprintCallable)
    void ToUESetString(const FString& str1, const FString& str2, const FString& vehicle_name = "");
    void ToUESetString_Implementation(const FString& str1, const FString& str2, const FString& vehicle_name = "");

    UFUNCTION(BlueprintNativeEvent, BlueprintCallable)
    bool GetVehicleAttackFromAPI( const FString& vehicle_name = "");
    bool GetVehicleAttackFromAPI_Implementation( const FString& vehicle_name = "");

    UFUNCTION(BlueprintNativeEvent, BlueprintCallable)
    void SetVehicleStateToAPI(bool isAlive, const FString& vehicle_name = "");
    void SetVehicleStateToAPI_Implementation(bool isAlive, const FString& vehicle_name = "");

    UPROPERTY()
    bool bIsApiControlled = false;

private:
    typedef msr::airlib::ClockFactory ClockFactory;
    typedef common_utils::Utils Utils;
    typedef msr::airlib::TTimePoint TTimePoint;
    typedef msr::airlib::TTimeDelta TTimeDelta;
    typedef ACarPawn TVehiclePawn;
    typedef msr::airlib::VehicleSimApiBase VehicleSimApiBase;
    typedef msr::airlib::VectorMath VectorMath;
    typedef msr::airlib::Vector3r Vector3r;

private:
    void initializePauseState();

protected:
    virtual void setupClockSpeed() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const override;
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const override;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const override;
    virtual PawnEvents* getVehiclePawnEvents(APawn* pawn) const override;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const override;
    virtual void initializeVehiclePawn(APawn* pawn) override;
    virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
        const PawnSimApi::Params& pawn_sim_api_params) const override;
    virtual msr::airlib::VehicleApiBase* getVehicleApi(const PawnSimApi::Params& pawn_sim_api_params,
                                                       const PawnSimApi* sim_api) const override;

private:
    std::atomic<float> current_clockspeed_;
    std::atomic<TTimeDelta> pause_period_;
    std::atomic<TTimePoint> pause_period_start_;
    uint32_t targetFrameNumber_;
    std::atomic_bool frame_countdown_enabled_;
    ;
};
