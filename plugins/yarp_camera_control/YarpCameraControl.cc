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
    stopRecording();
    _rpcServer.close();
    delete _callback;
}

/////////////////////////////////////////////////
void YarpCameraControl::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    _recordingNumber = 0;


    _isRecording = false;


    // Get the parent sensor.
    _cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!_cameraSensor)
    {
        gzerr << "YarpCameraControl requires a CameraSensor.\n";
        return;
    }


    _camera = _cameraSensor->GetCamera();
    _camera->EnableSaveFrame(true);

    // Connect to the sensor update event.
    _updateConnection = _cameraSensor->ConnectUpdated(std::bind(&YarpCameraControl::OnUpdate, this));

    // Make sure the parent sensor is active.
    _cameraSensor->SetActive(true);
    _cameraName = _cameraSensor->Name();

    _rpcServerPortName = "/gazebo/"+ _cameraName +"/rpc:i";
    _rpcServer.open(_rpcServerPortName);
    _callback = new RpcCallback(this);
    _rpcServer.setReader(*_callback);


}

/////////////////////////////////////////////////
void YarpCameraControl::OnUpdate()
{
    if(_isRecording) {
        auto filename = this->getFrameFilename();
        _camera->SaveFrame(filename);
    }
}
/////////////////////////////////////////////////

std::string YarpCameraControl::getFrameFilename()
{
    auto seconds_elapsed = yarp::os::Time::now() - _recordStartTime;
    return _imageSaveDir + "/" + std::to_string(seconds_elapsed) + ".png";
}

bool YarpCameraControl::startRecording()
{
    if(!_isRecording) {
        _recordStartTime = yarp::os::Time::now();
        _isRecording = true;
        ++_recordingNumber;
        return true;
    }
    return false;
}

bool YarpCameraControl::stopRecording()
{
    if(_isRecording) {
        _isRecording = false;
        generateVideoFromImages();
        return true;
    }
    return false;
}

void YarpCameraControl::generateVideoFromImages()
{

}

void YarpCameraControl::parseAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto tag = in.get(0).asString();
    if ( tag == "record" ) {
        parseRecordMessage(in, out);
    } else if ( tag == "get_pose" ) {
        parseGetPoseMessage(in, out);
    } else if ( tag == "set_resolution" ) {
        parseSetResolutionMessage(in, out);
    } else if ( tag == "help" ) {
        parseHelpMessage(in, out);
    } else {
        parseHelpMessage(in, out);
    }
}

void YarpCameraControl::parseRecordMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    if ( in.size() >= 2 ) {
        bool start = in.get(1).asBool();
        if ( in.size() >= 3 ) {
            setSaveDir( in.get(2).asString() );
            if ( in.size() == 4 ) {
                setVideoName( in.get(3).asString() );
            } else {
                setVideoName(_cameraName + std::to_string(_recordingNumber));
            }
        } else {
            setSaveDir(boost::filesystem::current_path().string());
            setVideoName(_cameraName + std::to_string(_recordingNumber));
        }
        if( start ) {
            if( startRecording() ) {
                out.addInt(SUCCESS);
                out.addString(_saveDir);
                out.addString(_imageSaveDir);
                out.addString(_videoName);
                std::cout << "Recording started." << '\n';
                std::cout << "Files will be saved to: " << _saveDir << '\n';
                std::cout << "Stills will be saved to: " << _imageSaveDir << '\n';
                std::cout << "Video title will be: " << _videoName << '\n';
            } else {
                out.addInt(FAILURE);
            }
        } else {
            if ( stopRecording() ) {
                out.addInt(SUCCESS);
                out.addString(_saveDir);
                out.addString(_imageSaveDir);
                out.addString(_videoName);
                std::cout << "Recording stopped." << '\n';
                std::cout << "Files saved to: " << _saveDir << '\n';
                std::cout << "Stills saved to: " << _imageSaveDir << '\n';
                std::cout << "Video title: " << _videoName << '\n';
            } else {
                out.addInt(FAILURE);
            }
        }
    } else {
        out.addInt(FAILURE);
    }
}

void YarpCameraControl::setSaveDir(const std::string& newSaveDir)
{
    auto newDir = boost::filesystem::path(newSaveDir);
    if ( ! boost::filesystem::exists( newDir ) ) {
        boost::filesystem::create_directories(newDir);
    }
    _saveDir = newSaveDir;
}

void YarpCameraControl::setVideoName(const std::string& newVideoName)
{
    _videoName = newVideoName;
    std::string extension(".mp4");
    if ( newVideoName.find(extension) == std::string::npos ) {
        _videoName += extension;
    }
    createImageSaveDir( _videoName.substr( 0, _videoName.size()-4 ) );
}

void YarpCameraControl::createImageSaveDir(const std::string& newImageSaveDir)
{
    _imageSaveDir = _saveDir + "/" + newImageSaveDir + "_images";
    auto newDir = boost::filesystem::path(_imageSaveDir);
    if ( ! boost::filesystem::exists( newDir ) ) {
        boost::filesystem::create_directories(newDir);
    } else {
        if ( !boost::filesystem::is_empty(newDir) ) {
            std::cout << "WARNING: Overwriting images in " << newDir << ". Old data will be lost." << '\n';

            boost::filesystem::recursive_directory_iterator rdi(newDir);
            boost::filesystem::recursive_directory_iterator end_rdi;
            std::string extension(".png");
            for (; rdi != end_rdi; ++rdi) {
                if (extension.compare(rdi->path().extension().string()) == 0) {
                    try {
                        if( boost::filesystem::is_regular_file(rdi->status()) ) {
                            boost::filesystem::remove(rdi->path());
                        }
                    } catch (const std::exception& ex ) {
                        ex;
                    }
                }
            }
        }
    }

}

void YarpCameraControl::parseGetPoseMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    auto pose = _cameraSensor->Pose();
    auto pos = pose.Pos();
    auto rot = pose.Rot().Euler();
    out.addDouble(pos[0]); // x
    out.addDouble(pos[1]); // y
    out.addDouble(pos[2]); // z
    out.addDouble(rot[0]); // roll
    out.addDouble(rot[1]); // pitch
    out.addDouble(rot[2]); // yaw
}

void YarpCameraControl::parseSetResolutionMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    int width, height;
    if( in.size()>=2 ) {
        width = in.get(1).asInt();
        if ( in.size()==3 ) {
            height = in.get(2).asInt();
        } else {
            // If height is not given then provide in 16:9 ratio
            height = std::round(width * 9 / 16);
        }
    } else {
        out.addInt(FAILURE);
        return;
    }
    _camera->SetImageSize(width, height);
    if ( (width == _camera->GetImageWidth()) && (height == _camera->GetImageHeight()) ) {
        out.addInt(SUCCESS);
        out.addInt(width);
        out.addInt(height);
        std::cout << "Recording video in " << width << " x " << height << " resolution." << '\n';
    } else {
        out.addInt(FAILURE);
        std::cout << "Failed to set new resolution." << '\n';
    }

}
void YarpCameraControl::parseHelpMessage(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{

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
