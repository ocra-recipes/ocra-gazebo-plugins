#include "YarpCameraControl.hh"

using namespace gazebo;



GZ_REGISTER_SENSOR_PLUGIN(YarpCameraControl)

/////////////////////////////////////////////////
YarpCameraControl::YarpCameraControl() : SensorPlugin()
{
}

/////////////////////////////////////////////////
YarpCameraControl::~YarpCameraControl()
{
    _rpcServer.close();
    delete _callback;
}

/////////////////////////////////////////////////
void YarpCameraControl::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    _isRecording = false;


    // Get the parent sensor.
    _cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!_cameraSensor)
    {
        gzerr << "YarpCameraControl requires a CameraSensor.\n";
        return;
    }

    _imageSavePath = "/home/ryan/tmp_recordings";

    _camera = _cameraSensor->GetCamera();
    _camera->EnableSaveFrame(true);

    // Connect to the sensor update event.
    _updateConnection = _cameraSensor->ConnectUpdated(std::bind(&YarpCameraControl::OnUpdate, this));

    // Make sure the parent sensor is active.
    _cameraSensor->SetActive(true);

    _rpcServerPortName = "/camera/test/rpc:i";
    _rpcServer.open(_rpcServerPortName);
    _callback = new RpcCallback(this);
    _rpcServer.setReader(*_callback);
}

/////////////////////////////////////////////////
void YarpCameraControl::OnUpdate()
{
    if(_isRecording) {
        auto filename = this->getFrameFilename();
        std::cout << "filename: " + filename << std::endl;
        _camera->SaveFrame(filename);
    }
}

std::string YarpCameraControl::getFrameFilename()
{
    auto seconds_elapsed = yarp::os::Time::now() - _recordStartTime;

    return _imageSavePath + "/" + std::to_string(seconds_elapsed) + ".jpg";
}

bool YarpCameraControl::startRecording()
{
    if(!_isRecording) {
        _recordStartTime = yarp::os::Time::now();
        _isRecording = true;
        std::cout << "Start Recording." << std::endl;
        return true;
    }
    return false;
}

bool YarpCameraControl::stopRecording()
{
    if(_isRecording) {
        _isRecording = false;
        generateVideoFromImages();
        std::cout << "Stop Recording." << std::endl;
        return true;
    }
    return false;
}

void YarpCameraControl::generateVideoFromImages()
{

}


void YarpCameraControl::parseAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    std::string tag = in.get(0).asString();
    if(tag == "record") {
        bool worked;
        if( in.get(1).asBool() ) {
            worked = startRecording();
        } else {
            worked = stopRecording();
        }
        out.addInt(worked);
    }
}




////////////////////////////////////////////////////////////////////////////////////////////
YarpCameraControl::RpcCallback::RpcCallback(YarpCameraControl* camCtrl)
: _camCtrl{camCtrl}
{

}

bool YarpCameraControl::RpcCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle in, out;
    bool ok = in.read(connection);
    if (!ok) return false;
    _camCtrl->parseAndReply(in, out);
    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender!=NULL) {
        out.write(*returnToSender);
    }

    return true;
}
