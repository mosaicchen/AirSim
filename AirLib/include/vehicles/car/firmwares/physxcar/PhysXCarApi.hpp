// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_PhysXCarController_hpp
#define msr_airlib_PhysXCarController_hpp

#include "vehicles/car/api/CarApiBase.hpp"

namespace msr
{
namespace airlib
{

    class PhysXCarApi : public CarApiBase
    {
    public:
        PhysXCarApi(const AirSimSettings::VehicleSetting* vehicle_setting, std::shared_ptr<SensorFactory> sensor_factory,
                    const Kinematics::State& state, const Environment& environment)
            : CarApiBase(vehicle_setting, sensor_factory, state, environment), home_geopoint_(environment.getHomeGeoPoint())
        {
        }

        ~PhysXCarApi()
        {
        }

    protected:
        virtual void resetImplementation() override
        {
            CarApiBase::resetImplementation();
        }

    public:
        virtual void update() override
        {
            CarApiBase::update();
        }

        virtual const SensorCollection& getSensors() const override
        {
            return CarApiBase::getSensors();
        }

        // VehicleApiBase Implementation
        virtual void enableApiControl(bool is_enabled) override
        {
            if (api_control_enabled_ != is_enabled) {
                last_controls_ = CarControls();
                api_control_enabled_ = is_enabled;
            }
        }

        virtual bool isApiControlEnabled() const override
        {
            return api_control_enabled_;
        }

        virtual GeoPoint getHomeGeoPoint() const override
        {
            return home_geopoint_;
        }

        virtual bool armDisarm(bool arm) override
        {
            //TODO: implement arming for car
            unused(arm);
            return true;
        }

    public:
        virtual void setCarControls(const CarControls& controls) override
        {
            last_controls_ = controls;
        }

        virtual void updateCarState(const CarState& car_state) override
        {
            last_car_state_ = car_state;
        }

        virtual const CarState& getCarState() const override
        {
            return last_car_state_;
        }

        virtual const CarControls& getCarControls() const override
        {
            return last_controls_;
        }

        virtual void setCarMCMsg(const CarMCMsg& msg) override
        {
            recvMCMsg_= msg;
        }

        virtual const CarMCMsg& getCarMCMsg() const override
        {
            return sentMCMsg_;
        }

        virtual void sendCarMCMsg(const CarMCMsg& msg) override
        {
            sentMCMsg_ = msg;
        }

        virtual const CarMCMsg getRecvCarMCMsg() const override
        {
            return recvMCMsg_;
        }
        virtual const CarMCMsg getSentCarMCMsg() const override
        {
            return sentMCMsg_;
        }
        
        virtual void toUECarCustomString(const CustomStrData& data) override
        {
            toUECustomStr_ = data;
        }
        virtual const CustomStrData& fromUECarCustomString() const override
        {
            return fromUECustomStr_;
        }
        
        
        virtual void setAttack(const AttackState& data) override
        {
            attackState_ = data;
        }
        
        virtual const AttackState& getAttackState() const override
        {
            return attackState_;
        }

        virtual const VehicleState& getVehicleState() const override
        {
            return vehicleState_;
        }
        
        virtual void updateVehicleState(const VehicleState state) override
        {
            vehicleState_ = state;
        }

    private:
        bool api_control_enabled_ = false;
        GeoPoint home_geopoint_;
        CarControls last_controls_;
        CarState last_car_state_;
        CarMCMsg recvMCMsg_;
        CarMCMsg sentMCMsg_;
        CustomStrData toUECustomStr_;
        CustomStrData fromUECustomStr_;
        AttackState attackState_;
        VehicleState vehicleState_;
    };
}
}

#endif
