// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_CarApiBase_hpp
#define air_CarApiBase_hpp

#include "common/VectorMath.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "physics/Kinematics.hpp"
#include "sensors/SensorBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/SensorFactory.hpp"

namespace msr
{
namespace airlib
{

    class CarApiBase : public VehicleApiBase
    {
    public:
        struct CarControls
        {
            float throttle = 0; /* 1 to -1 */
            float steering = 0; /* 1 to -1 */
            float brake = 0; /* 1 to -1 */
            bool handbrake = false;
            bool is_manual_gear = false;
            int manual_gear = 0;
            bool gear_immediate = true;

            CarControls()
            {
            }
            CarControls(float throttle_val, float steering_val, float brake_val, bool handbrake_val,
                        bool is_manual_gear_val, int manual_gear_val, bool gear_immediate_val)
                : throttle(throttle_val), steering(steering_val), brake(brake_val), handbrake(handbrake_val), is_manual_gear(is_manual_gear_val), manual_gear(manual_gear_val), gear_immediate(gear_immediate_val)
            {
            }
            void set_throttle(float throttle_val, bool forward)
            {
                if (forward) {
                    is_manual_gear = false;
                    manual_gear = 0;
                    throttle = std::abs(throttle_val);
                }
                else {
                    is_manual_gear = false;
                    manual_gear = -1;
                    throttle = -std::abs(throttle_val);
                }
            }
        };

        struct CarState
        {
            float speed;
            int gear;
            float rpm;
            float maxrpm;
            bool handbrake;
            Kinematics::State kinematics_estimated;
            uint64_t timestamp;

            CarState()
            {
            }

            CarState(float speed_val, int gear_val, float rpm_val, float maxrpm_val, bool handbrake_val,
                     const Kinematics::State& kinematics_estimated_val, uint64_t timestamp_val)
                : speed(speed_val), gear(gear_val), rpm(rpm_val), maxrpm(maxrpm_val), handbrake(handbrake_val), kinematics_estimated(kinematics_estimated_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const Vector3r& getPosition() const
            {
                return kinematics_estimated.pose.position;
            }
            const Quaternionr& getOrientation() const
            {
                return kinematics_estimated.pose.orientation;
            }
        };

        struct CarMCMsg
        {
            string msg;
            uint64_t timestamp;

            CarMCMsg()
            {
            }

            CarMCMsg(string msg_val, uint64_t timestamp_val)
                :  msg(msg_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const string& getMsg() const
            {
                return msg;
            }
        };

        struct CustomFloatData
        {
            float value;
            uint64_t timestamp;

            CustomFloatData()
            {
            }

            CustomFloatData(float float_val, uint64_t timestamp_val)
                :  value(float_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const float& getValue() const
            {
                return value;
            }
        };

        struct CustomStrData
        {
            string str1;
            string str2;
            uint64_t timestamp;

            CustomStrData()
            {
            }

            CustomStrData(string str_val1, string str_val2, uint64_t timestamp_val)
                :  str1(str_val1),str2(str_val2), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const string& getStr1() const
            {
                return str1;
            }
            const string& getStr2() const
            {
                return str2;
            }
        };

        struct AttackState
        {
            bool isAttack;
            uint64_t timestamp;

            AttackState()
            {
            }

            AttackState(bool attack_val, uint64_t timestamp_val)
                :  isAttack(attack_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const bool& getIsAttack() const
            {
                return isAttack;
            }
        };

        struct VehicleState
        {
            bool state = true;
            uint64_t timestamp;

            VehicleState()
            {
            }

            VehicleState(bool state_val, uint64_t timestamp_val)
                :  state(state_val), timestamp(timestamp_val)
            {
            }

            //shortcuts
            const bool& getState() const
            {
                return state;
            }
        };

    public:
        // TODO: Temporary constructor for the Unity implementation which does not use the new Sensor Configuration Settings implementation.
        //CarApiBase() {}

        CarApiBase(const AirSimSettings::VehicleSetting* vehicle_setting,
                   std::shared_ptr<SensorFactory> sensor_factory,
                   const Kinematics::State& state, const Environment& environment)
        {
            initialize(vehicle_setting, sensor_factory, state, environment);
        }

        virtual void update() override
        {
            VehicleApiBase::update();

            getSensors().update();
        }

        void reportState(StateReporter& reporter) override
        {
            getSensors().reportState(reporter);
        }

        // sensor helpers
        virtual const SensorCollection& getSensors() const override
        {
            return sensors_;
        }

        SensorCollection& getSensors()
        {
            return sensors_;
        }

        void initialize(const AirSimSettings::VehicleSetting* vehicle_setting,
                        std::shared_ptr<SensorFactory> sensor_factory,
                        const Kinematics::State& state, const Environment& environment)
        {
            sensor_factory_ = sensor_factory;

            sensor_storage_.clear();
            sensors_.clear();

            addSensorsFromSettings(vehicle_setting);

            getSensors().initialize(&state, &environment);
        }

        void addSensorsFromSettings(const AirSimSettings::VehicleSetting* vehicle_setting)
        {
            const auto& sensor_settings = vehicle_setting->sensors;

            sensor_factory_->createSensorsFromSettings(sensor_settings, sensors_, sensor_storage_);
        }

        virtual void setCarControls(const CarControls& controls) = 0;
        virtual void updateCarState(const CarState& state) = 0;
        virtual const CarState& getCarState() const = 0;
        virtual const CarControls& getCarControls() const = 0;

        virtual void setCarMCMsg(const CarMCMsg& msg) = 0;
        virtual const CarMCMsg& getCarMCMsg() const = 0;

        virtual void sendCarMCMsg(const CarMCMsg& msg) = 0;
        virtual const CarMCMsg getRecvCarMCMsg() const = 0;
        virtual const CarMCMsg getSentCarMCMsg() const = 0;

        virtual void toUECarCustomString(const CustomStrData& data) = 0;
        virtual const CustomStrData& fromUECarCustomString() const = 0;

        virtual void setAttack(const AttackState& data) = 0;
        virtual const AttackState& getAttackState() const = 0;

        virtual const VehicleState& getVehicleState() const = 0;
        virtual void updateVehicleState(const VehicleState state) = 0;


        virtual ~CarApiBase() = default;

        std::shared_ptr<const SensorFactory> sensor_factory_;
        SensorCollection sensors_; //maintains sensor type indexed collection of sensors
        vector<shared_ptr<SensorBase>> sensor_storage_; //RAII for created sensors

    protected:
        virtual void resetImplementation() override
        {
            //reset sensors last after their ground truth has been reset
            getSensors().reset();
        }
    };
}
} //namespace
#endif
