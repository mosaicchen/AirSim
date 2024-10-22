#pragma once

#include "CoreMinimal.h"
#include "ChaosWheeledVehicleMovementComponent.h"
#include "WheeledVehiclePawn.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "UObject/ConstructorHelpers.h"

#include "physics/Kinematics.hpp"
#include "vehicles/car/api/CarApiBase.hpp"
#include "common/AirSimSettings.hpp"
#include "AirBlueprintLib.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "PawnEvents.h"
#include "PIPCamera.h"

#include "CarPawn.generated.h"

class UPhysicalMaterial;
class UCameraComponent;
class USpringArmComponent;
class UTextRenderComponent;
class UInputComponent;
class UAudioComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FIniticlizeDelegateSignature);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_FiveParams(FSetActorMoveDelegate, FVector, in_Point, FRotator, in_rotator, float, LWheelSpeed, float, RWheelSpeed, bool, in_Phsics);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FMoveInputDelegate, float, in_FSpeed, float, in_RSpeed);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FMoveUVAInputDelegate, float, in_FSpeed);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRightUVAInputDelegate, float, in_FSpeed);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FUpUVAInputDelegate, float, in_FSpeed);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FRLUVAInputDelegate, float, in_FSpeed);

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FApiControlEvent);


UCLASS(config = Game)
class ACarPawn : public AWheeledVehiclePawn
{
	GENERATED_BODY()

public:
	ACarPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float Delta) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
		FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

	//interface
	void initializeForBeginPlay(bool engine_sound);
	const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;
	PawnEvents* getPawnEvents()
	{
		return &pawn_events_;
	}
	UChaosVehicleMovementComponent* getVehicleMovementComponent() const;
	const msr::airlib::CarApiBase::CarControls& getKeyBoardControls() const
	{
		return keyboard_controls_;
	}

	UPROPERTY(BlueprintAssignable)
	FIniticlizeDelegateSignature IniticlizeDelegate;
	UPROPERTY(BlueprintAssignable)
	FSetActorMoveDelegate SetActorMoveDelegate;
	UPROPERTY(BlueprintAssignable)
	FMoveInputDelegate MoveInputDelegate;
	//UVAMove
	UPROPERTY(BlueprintAssignable)
	FMoveUVAInputDelegate UAVMoveInputDelegate;
	UPROPERTY(BlueprintAssignable)
	FRightUVAInputDelegate UAVRightInputDelegate;
	UPROPERTY(BlueprintAssignable)
	FUpUVAInputDelegate UAVUpInputDelegate;
	UPROPERTY(BlueprintAssignable)
	FRLUVAInputDelegate UAVRLInputDelegate;

	UPROPERTY(BlueprintAssignable)
	FApiControlEvent ApiControlEvent;
	//Í¬²½ÓÃ
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	bool WreckState;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float RotatorSpeed;
	//
	UPROPERTY(BlueprintReadWrite)
	int Id;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	int Type;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	int ObjectType;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float lookDistance;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	bool inputOff;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float RotateSpeed;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	bool UAVinput;

	UPROPERTY(BlueprintReadWrite)
	bool InputOpen;
	//TArray<FInputAxisBinding*>
	UFUNCTION(BlueprintCallable)
	void onMoveForward(float Val);
	UFUNCTION(BlueprintCallable)
	void onMoveRight(float Val);

	UFUNCTION(BlueprintCallable)
	void SetSimulatePhysics(bool Open);
	UFUNCTION(BlueprintCallable)
	void SetMoveData(FVector in_Point, FRotator in_rotator, float LWheelSpeed, float RWheelSpeed, bool in_Phsics);
	UFUNCTION(BlueprintCallable, BlueprintPure)
	void GetData(int& out_id, int& out_objectId, FVector& pos, FRotator& Rot, float& LWheelSpeed, float& RWheelSpeed, float& out_rotatorSpeed, bool& state);

	//PID
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float TestMoveSpeed;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKp;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKi;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKd;
	float MoveInputV;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float TestRotatorSpeed;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKpR;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKiR;
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float InKdR;

	//Test
	//UPROPERTY(BlueprintReadWrite, EditAnywhere)
	//float INPLTT;

	//
	void onReversePressed();
	void onReverseReleased();
	void onUpMove(float Val);
	void onRLMove(float Val);

private:
	void updateHUDStrings();
	void setupVehicleMovementComponent();
	void updateInCarHUD();
	void updatePhysicsMaterial();

	void setupInputBindings();
	/*
		void onMoveForward(float Val);
		void onMoveRight(float Val);
	*/
	void onHandbrakePressed();
	void onHandbrakeReleased();
	UFUNCTION(BlueprintCallable)
	void onFootBrake(float Val);
	//PID
	void onTestMoveForward(float val);

private:
	typedef msr::airlib::AirSimSettings AirSimSettings;

	UClass* pip_camera_class_;

	PawnEvents pawn_events_;

	bool is_low_friction_;
	UPhysicalMaterial* slippery_mat_;
	UPhysicalMaterial* non_slippery_mat_;

	UPROPERTY()
	USceneComponent* camera_front_center_base_;
	UPROPERTY()
	USceneComponent* camera_front_left_base_;
	UPROPERTY()
	USceneComponent* camera_front_right_base_;
	UPROPERTY()
	USceneComponent* camera_driver_base_;
	UPROPERTY()
	USceneComponent* camera_back_center_base_;

	UPROPERTY()
	APIPCamera* camera_front_center_;
	UPROPERTY()
	APIPCamera* camera_front_left_;
	UPROPERTY()
	APIPCamera* camera_front_right_;
	UPROPERTY()
	APIPCamera* camera_driver_;
	UPROPERTY()
	APIPCamera* camera_back_center_;

	UTextRenderComponent* speed_text_render_;
	UTextRenderComponent* gear_text_render_;
	UAudioComponent* engine_sound_audio_;

	msr::airlib::CarApiBase::CarControls keyboard_controls_;

	FText last_speed_;
	FText last_gear_;
	FColor last_gear_display_color_;
	FColor last_gear_display_reverse_color_;
};
