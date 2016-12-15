#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>

#include <yarp/os/all.h>

namespace gazebo
{


/// \brief An example plugin for a camera sensor.
class YarpCameraControl : public SensorPlugin
{
public:
    /// \brief Constructor.
    YarpCameraControl();
    /// \brief Destructor.
    virtual ~YarpCameraControl();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the camera sensor's update signal.
    virtual void OnUpdate();

    void parseAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out);


    class RpcCallback : public yarp::os::PortReader {

    public:
        RpcCallback(YarpCameraControl* camCtrl);
        virtual bool read(yarp::os::ConnectionReader& connection);
    private:
        YarpCameraControl* _camCtrl;
    };

private:
    /// \brief Pointer to the camera sensor
    sensors::CameraSensorPtr _cameraSensor;
    /// \brief Pointer to the camera
    rendering::CameraPtr _camera;
    /// \brief Connection that maintains a link between the camera sensor's
    /// updated signal and the OnUpdate callback.
    event::ConnectionPtr _updateConnection;


    std::string _imageSavePath;

    std::string _rpcServerPortName;
    yarp::os::RpcServer _rpcServer;
    yarp::os::Network _yarp;
    RpcCallback* _callback;
    bool _isRecording;
    double _recordStartTime;


private:
    std::string getFrameFilename();
    bool startRecording();
    bool stopRecording();
    void generateVideoFromImages();
};
}
#endif
