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

    // msr::airlib::CarApiBase::CarMCMsg getCarMCMsg() const;
    // void setCarMCMsg(const msr::airlib::CarApiBase::CarMCMsg& msg);

    // void sendCarMCMsg(msr::airlib::CarApiBase::CarMCMsg& msg)
    // {
    //     vehicle_api_->sendCarMCMsg(msg);
    // }

    // msr::airlib::CarApiBase::CarMCMsg getRecvCarMCMsg() const
    // {
    //     return vehicle_api_->getRecvCarMCMsg();
    // }
    // msr::airlib::CarApiBase::CarMCMsg getSentCarMCMsg() const
    // {
    //     return vehicle_api_->getSentCarMCMsg();
    // }

    void reset();
    void update();

    virtual ~CarPawnApi();

private:
    UChaosWheeledVehicleMovementComponent* movement_;
    msr::airlib::CarApiBase::CarControls last_controls_;
    ACarPawn* pawn_;
    const msr::airlib::Kinematics::State* pawn_kinematics_;
    msr::airlib::CarApiBase* vehicle_api_;
    
    // msr::airlib::CarApiBase::CarMCMsg recvMCMsg_;
    // msr::airlib::CarApiBase::CarMCMsg sentMCMsg_;
};
