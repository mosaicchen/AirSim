// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include precompiled header file first

#include "vehicles/car/api/CarRpcLibClient.hpp"

#include "common/Common.hpp"
#include "common/ClockFactory.hpp"
#include <thread>
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK

#ifdef nil
#undef nil
#endif // nil

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/client.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/car/api/CarRpcLibAdaptors.hpp"

STRICT_MODE_ON
#ifdef _MSC_VER
__pragma(warning(disable : 4239))
#endif

    namespace msr
{
    namespace airlib
    {

        typedef msr::airlib_rpclib::CarRpcLibAdaptors CarRpcLibAdaptors;

        CarRpcLibClient::CarRpcLibClient(const string& ip_address, uint16_t port, float timeout_sec)
            : RpcLibClientBase(ip_address, port, timeout_sec)
        {
        }

        CarRpcLibClient::~CarRpcLibClient()
        {
        }

        void CarRpcLibClient::setCarControls(const CarApiBase::CarControls& controls, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->call("setCarControls", CarRpcLibAdaptors::CarControls(controls), vehicle_name);
        }

        CarApiBase::CarState CarRpcLibClient::getCarState(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->call("getCarState", vehicle_name).as<CarRpcLibAdaptors::CarState>().to();
        }
        CarApiBase::CarControls CarRpcLibClient::getCarControls(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->call("getCarControls", vehicle_name).as<CarRpcLibAdaptors::CarControls>().to();
        }
        void CarRpcLibClient::setCarMCMsg(const CarApiBase::CarMCMsg& msg, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->call("setCarMCMsg", CarRpcLibAdaptors::CarMCMsg(msg), vehicle_name);
        }
        CarApiBase::CarMCMsg CarRpcLibClient::getCarMCMsg()
        {
            return static_cast<rpc::client*>(getClient())->call("getCarMCMsg").as<CarRpcLibAdaptors::CarMCMsg>().to();
        }
        
        void CarRpcLibClient::toUECarCustomString(const CarApiBase::CustomStrData& data, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->call("toUECarCustomString", CarRpcLibAdaptors::CustomStrData(data), vehicle_name);
        }
        CarApiBase::CustomStrData CarRpcLibClient::fromUECarCustomString()
        {
            return static_cast<rpc::client*>(getClient())->call("fromUECarCustomString").as<CarRpcLibAdaptors::CustomStrData>().to();
        }
        
        void CarRpcLibClient::setAttack(const CarApiBase::AttackState& data, const std::string& vehicle_name)
        {
            static_cast<rpc::client*>(getClient())->call("setAttack", CarRpcLibAdaptors::AttackState(data), vehicle_name);
        }
        CarApiBase::VehicleState CarRpcLibClient::getVehicleState(const std::string& vehicle_name)
        {
            return static_cast<rpc::client*>(getClient())->call("getVehicleState", vehicle_name).as<CarRpcLibAdaptors::VehicleState>().to();
        }

    }
} //namespace

#endif
#endif
