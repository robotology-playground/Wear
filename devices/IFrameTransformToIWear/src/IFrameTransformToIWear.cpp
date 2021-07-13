/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "IFrameTransformToIWear.h"

#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <mutex>
#include <string>
#include <vector>

const std::string DeviceName = "IFrameTransformToIWear";
const std::string LogPrefix = DeviceName + " :";

using namespace wearable;
using namespace wearable::sensor;
using namespace wearable::devices;

struct ParsedOptions
{
    wearable::WearableName wearableName;
    wearable::sensor::SensorType wearableSensorType;
    std::string rootFrameId;
};

class IFrameTransformToIWear::Impl
{
public:
    bool firstRun = true;
    mutable std::recursive_mutex mutex;

    TimeStamp timestamp;
    ParsedOptions options;

    // Sensors stored for exposing wearable::IWear
    std::map<std::string, std::shared_ptr<sensor::impl::PoseSensor>> poseSensors;
    std::map<std::string, std::shared_ptr<sensor::impl::VirtualLinkKinSensor>>
        virtualLinkKinSensors;

    bool allocateSensor(const wearable::sensor::SensorType type,
                        const wearable::sensor::SensorName name,
                        IAnalogSensorHandler handler);
};

// ================================
// WEARABLE SENSORS IMPLEMENTATIONS // TODO
// ================================

class ForceTorque6DSensor : public IForceTorque6DSensor
{
public:
    unsigned offset = 0;
    bool groundReactionFT;
    IAnalogSensorHandler handler;

    ForceTorque6DSensor(SensorName name, IAnalogSensorHandler analogSensorHandler, SensorStatus status = SensorStatus::Unknown)
        : IForceTorque6DSensor(name, status), handler(analogSensorHandler)
    {}

    void setStatus(const SensorStatus status) { m_status = status; }

    bool getForceTorque6D(Vector3& force3D, Vector3& torque3D) const override
    {
        // Dirty workaround to set the status from a const method and call non-const methods of the handler
        auto nonConstThis = const_cast<ForceTorque6DSensor*>(this);
        bool dataOk = nonConstThis->handler.readData();
        nonConstThis->setStatus(nonConstThis->handler.getStatus());

        // TODO: The positions of force and torques are hardcoded. Forces should be the first
        //       triplet of elements of the read vector and torques the second one.
        bool ok = dataOk && nonConstThis->handler.getData(force3D, offset) && nonConstThis->handler.getData(torque3D, offset + 3);
        if (groundReactionFT) {
            force3D[0] = -1 * force3D[0];
            force3D[1] = -1 * force3D[1];
            force3D[2] = -1 * force3D[2];

            torque3D[0] = -1 * torque3D[0];
            torque3D[1] = -1 * torque3D[1];
            torque3D[2] = -1 * torque3D[2];
        }
        return ok;
    }
};

// TODO: implement all the other sensors

// ======================
// IFrameTransformToIWear
// ======================

wearable::sensor::SensorType sensorTypeFromString(std::string sensorTypeString)
{
    if (sensorTypeString == "Accelerometer")
        return wearable::sensor::SensorType::Accelerometer;
    else if (sensorTypeString == "EmgSensor")
        return wearable::sensor::SensorType::EmgSensor;
    else if (sensorTypeString == "Force3DSensor")
        return wearable::sensor::SensorType::Force3DSensor;
    else if (sensorTypeString == "ForceTorque6DSensor")
        return wearable::sensor::SensorType::ForceTorque6DSensor;
    else if (sensorTypeString == "FreeBodyAccelerationSensor")
        return wearable::sensor::SensorType::FreeBodyAccelerationSensor;
    else if (sensorTypeString == "Gyroscope")
        return wearable::sensor::SensorType::Gyroscope;
    else if (sensorTypeString == "Magnetometer")
        return wearable::sensor::SensorType::Magnetometer;
    else if (sensorTypeString == "OrientationSensor")
        return wearable::sensor::SensorType::OrientationSensor;
    else if (sensorTypeString == "PoseSensor")
        return wearable::sensor::SensorType::PoseSensor;
    else if (sensorTypeString == "PositionSensor")
        return wearable::sensor::SensorType::PositionSensor;
    else if (sensorTypeString == "SkinSensor")
        return wearable::sensor::SensorType::SkinSensor;
    else if (sensorTypeString == "TemperatureSensor")
        return wearable::sensor::SensorType::TemperatureSensor;
    else if (sensorTypeString == "Torque3DSensor")
        return wearable::sensor::SensorType::Torque3DSensor;
    else if (sensorTypeString == "VirtualLinkKinSensor")
        return wearable::sensor::SensorType::VirtualLinkKinSensor;
    else if (sensorTypeString == "VirtualJointKinSensor")
        return wearable::sensor::SensorType::VirtualJointKinSensor;
    else if (sensorTypeString == "VirtualSphericalJointKinSensor")
        return wearable::sensor::SensorType::VirtualSphericalJointKinSensor;
    else {
        yError() << LogPrefix << "Sensor type" << sensorTypeString << "not supported";
        return {};
    }
}

IFrameTransformToIWear::IFrameTransformToIWear()
    : pImpl{new Impl()}
{}

// Without this destructor here, the linker complains for
// undefined reference to vtable
IFrameTransformToIWear::~IFrameTransformToIWear() = default;

bool IFrameTransformToIWear::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("wearableName") && config.find("wearableName").isString())) {
        yError() << LogPrefix << "Parameter 'wearableName' missing or invalid";
        return false;
    }

    if (!(config.check("rootFrameID") && config.find("rootFrameID").isString())) {
        yError() << LogPrefix << "Parameter 'rootFrameID' missing or invalid";
        return false;
    }

    if (!(config.check("wearableSensorType") && config.find("wearableSensorType").isString())) {
        yError() << LogPrefix << "Parameter 'wearableSensorType' missing or invalid";
        return false;
    }

    // ===============
    // READ PARAMETERS
    // ===============

    pImpl->options.wearableName = config.find("wearableName").asString();
    pImpl->options.rootFrameID = config.find("rootFrameID").asString();
    std::string sensorType = config.find("wearableSensorType").asString();

    yInfo() << LogPrefix << "*** ====================";
    yInfo() << LogPrefix << "*** Wearable name      :" << pImpl->options.wearableName;
    yInfo() << LogPrefix << "*** Sensor Type        :" << sensorType;
    yInfo() << LogPrefix << "*** Root Frame ID      :" << pImpl->options.rootFrameID;
    yInfo() << LogPrefix << "*** ====================";

    // =====================
    // INITIALIZE THE DEVICE
    // =====================

    return true;
}

bool IFrameTransformToIWear::close()
{
    detach();
    pImpl->iSensor.reset();
    return true;
}

yarp::os::Stamp IFrameTransformToIWear::getLastInputStamp()
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);
    return yarp::os::Stamp(pImpl->timestamp.sequenceNumber, yarp::os::Time::now());
}

// TODO
bool IFrameTransformToIWear::Impl::allocateSensor(const wearable::sensor::SensorType type,
                                                const wearable::sensor::SensorName name,
                                                IAnalogSensorHandler handler)
{
    // The sensors are initialized as Ok in order to trigger the first data read.
    // If there is any error during the first read, the sensor updates its own status
    // that is then propagated to the global IWear status.
    switch (type) {
        case wearable::sensor::SensorType::Force3DSensor: {
            auto sensor = std::make_shared<Force3DSensor>(name, handler, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::ForceTorque6DSensor: {
            auto sensor = std::make_shared<ForceTorque6DSensor>(name, handler, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            sensor->groundReactionFT = options.getGroundReactionFT;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::TemperatureSensor: {
            auto sensor = std::make_shared<TemperatureSensor>(name, handler, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::Torque3DSensor: {
            auto sensor = std::make_shared<Torque3DSensor>(name, handler, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        case wearable::sensor::SensorType::SkinSensor: {
            auto sensor = std::make_shared<SkinSensor>(name, handler, SensorStatus::Ok);
            sensor->offset = options.channelOffset;
            iSensor = std::dynamic_pointer_cast<ISensor>(sensor);
            break;
        }
        default:
            // TODO: implement the remaining sensors
            return false;
    }

    return true;
}

// TODO
bool IFrameTransformToIWear::attach(yarp::dev::PolyDriver* poly)
{
    IAnalogSensorHandler handler;
    handler.buffer.resize(pImpl->options.numberOfChannels);

    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (!(poly->view(handler.interface) && handler.interface)) {
        yError() << LogPrefix << "Failed to view the IAnalogSensor interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    if (handler.interface->getChannels() == 0) {
        yError() << LogPrefix << "The number of channels is 0";
        return false;
    }

    if (handler.interface->getChannels()
        != pImpl->options.numberOfChannels + pImpl->options.channelOffset) {
        yError() << LogPrefix << "The number of sensor channels ("
                 << handler.interface->getChannels()
                 << ") is different than the number specified in the options plus the offset ("
                 << pImpl->options.numberOfChannels + pImpl->options.channelOffset << ")";
        return false;
    }

    for (unsigned i = 0; i < handler.interface->getChannels(); ++i) {
        if (handler.interface->getState(i) != yarp::dev::IAnalogSensor::AS_OK) {
            yError() << LogPrefix << "The status of IAnalogSensor interface for channel" << i
                     << "is not AS_OK (" << handler.interface->getState(i) << ")";
            return false;
        }
    }

    if (!pImpl->allocateSensor(pImpl->options.wearableSensorType, pImpl->options.sensorName, handler)) {
        yError() << LogPrefix << "Failed to allocate a new sensor of the specified type";
        return false;
    }

    // Notify that the sensor is ready to be used
    pImpl->firstRun = false;

    return true;
}

bool IFrameTransformToIWear::detach()
{
    return true;
}

bool IFrameTransformToIWear::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool IFrameTransformToIWear::detachAll()
{
    return detach();
}

wearable::SensorPtr<const ISensor> IFrameTransformToIWear::getSensor(const SensorName name) const
{
    // This device can provide only one sensor. Check if the name matches.
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return pImpl->iSensor;
}

wearable::VectorOfSensorPtr<const ISensor>
IFrameTransformToIWear::getSensors(const SensorType type) const
{
    wearable::VectorOfSensorPtr<const ISensor> vector;

    if (pImpl->options.wearableSensorType == type) {
        vector.push_back(getSensor(pImpl->options.sensorName));
    }

    return vector;
}

wearable::WearableName IFrameTransformToIWear::getWearableName() const
{
    return pImpl->options.wearableName + wearable::Separator;
}

wearable::WearStatus IFrameTransformToIWear::getStatus() const
{
    // This is necessary if something that uses the exposed IWear interface asks the status
    // before IAnalogSensor is attached
    if (pImpl->firstRun) {
        return WearStatus::WaitingForFirstRead;
    }

    if (!pImpl->iSensor) {
        yError() << LogPrefix << "The stored ISensor has not been yet allocated";
        return WearStatus::Error;
    }

    return pImpl->iSensor->getSensorStatus();
}

wearable::TimeStamp IFrameTransformToIWear::getTimeStamp() const
{
    std::lock_guard<std::recursive_mutex> lock(pImpl->mutex);

    pImpl->timestamp.sequenceNumber = 0; // Always zero
    pImpl->timestamp.time = yarp::os::Time::now();

    return pImpl->timestamp;
}

wearable::SensorPtr<const wearable::sensor::IAccelerometer>
IFrameTransformToIWear::getAccelerometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IForce3DSensor>
IFrameTransformToIWear::getForce3DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::IForce3DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>
IFrameTransformToIWear::getForceTorque6DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::IForceTorque6DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IGyroscope>
IFrameTransformToIWear::getGyroscope(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IMagnetometer>
IFrameTransformToIWear::getMagnetometer(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IOrientationSensor>
IFrameTransformToIWear::getOrientationSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ITemperatureSensor>
IFrameTransformToIWear::getTemperatureSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::ITemperatureSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::ITorque3DSensor>
IFrameTransformToIWear::getTorque3DSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::ITorque3DSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IEmgSensor>
IFrameTransformToIWear::getEmgSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IFreeBodyAccelerationSensor>
IFrameTransformToIWear::getFreeBodyAccelerationSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPoseSensor>
IFrameTransformToIWear::getPoseSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IPositionSensor>
IFrameTransformToIWear::getPositionSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::ISkinSensor>
IFrameTransformToIWear::getSkinSensor(const wearable::sensor::SensorName name) const
{
    if (!(pImpl->iSensor && (pImpl->iSensor->getSensorName() == name))) {
        yError() << LogPrefix << "Failed to get sensor" << name;
        return nullptr;
    }

    return std::dynamic_pointer_cast<const wearable::sensor::ISkinSensor>(pImpl->iSensor);
}

wearable::SensorPtr<const wearable::sensor::IVirtualLinkKinSensor>
IFrameTransformToIWear::getVirtualLinkKinSensor(const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualJointKinSensor>
IFrameTransformToIWear::getVirtualJointKinSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}

wearable::SensorPtr<const wearable::sensor::IVirtualSphericalJointKinSensor>
IFrameTransformToIWear::getVirtualSphericalJointKinSensor(
    const wearable::sensor::SensorName /*name*/) const
{
    return nullptr;
}
