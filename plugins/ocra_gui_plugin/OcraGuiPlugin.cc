#include "OcraGuiPlugin.hh"

using namespace gazebo;

OcraGuiPlugin::OcraGuiPlugin()
{
    // do nothing
}

OcraGuiPlugin::~OcraGuiPlugin()
{
    port.close();
}

void OcraGuiPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr)
{
    this->world = _parent;
    port.open("/Gazebo/OcraGuiPlugin/rpc:i");
    callback = RpcServerCallback(this);
    port.setReader(callback);

    robotWorldPose = math::Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    getRobotWorldPose();
}

void OcraGuiPlugin::getRobotWorldPose()
{
    if (this->world->GetModel("iCub")) {
        if (this->world->GetModel("iCub")->GetChildLink("l_foot")) {
            robotWorldPose = this->world->GetModel("iCub")->GetChildLink("l_foot")->GetWorldPose();
            std::cout << "Found iCub.l_foot. Getting robot world pose from this link." << std::endl;
        }
    }
    std::cout << "=========================================" << std::endl;
    std::cout << "-- OcraGuiPlugin --" << std::endl;
    std::cout << "Robot world pose:\n" << robotWorldPose << std::endl;
    std::cout << "=========================================" << std::endl;
}

void OcraGuiPlugin::parseInputAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out)
{
    bool worked = true;
    std::string message = "";
    std::string tag = in.get(0).asString();
    if (tag=="addTaskFrames") {
        addTaskFrames(in.get(1).asString(), worked, message);
    } else if (tag=="removeTaskFrames") {
        removeTaskFrames(in.get(1).asString(), worked, message);
    } else if (tag=="addSdfToWorld"){
        sdf::SDF modelSdf;
        modelSdf.SetFromString(in.get(1).asString());
        addSdfToWorld(modelSdf);
    } else if (tag=="getRobotWorldPose") {
        double x = robotWorldPose.pos.x;
        double y = robotWorldPose.pos.y;
        double z = robotWorldPose.pos.z;
        double qw = robotWorldPose.rot.w;
        double qx = robotWorldPose.rot.x;
        double qy = robotWorldPose.rot.y;
        double qz = robotWorldPose.rot.z;
        out.addDouble(x);
        out.addDouble(y);
        out.addDouble(z);
        out.addDouble(qw);
        out.addDouble(qx);
        out.addDouble(qy);
        out.addDouble(qz);
        message = "Robot world frame has the following coordinates relative to the Gazebo world frame: " + std::to_string(x) +", "+ std::to_string(y) +", "+ std::to_string(z) +", "+ std::to_string(qw) +", "+ std::to_string(qx) +", "+ std::to_string(qy) +", "+ std::to_string(qz) + ".";
    }else {
        // out.addInt(false);
    }
    out.addInt(worked);
    out.addString(message);
}

void OcraGuiPlugin::addSdfToWorld(const sdf::SDF& modelSDF)
{
    this->world->InsertModelSDF(modelSDF);
}

void OcraGuiPlugin::addTaskFrames(const std::string& taskName, bool& worked, std::string& message)
{
    if (checkIfTaskFramesExist(taskName)) {
        message = "Task frames for " + taskName + " are already in the gazebo world.";
        worked = false;
    } else {
        std::string taskFrameName = taskName + "-Frame";
        double frameTransparency = 0.3;
        std::string taskTargetName = taskName + "-Target";
        double targetTransparency = 0.6;

        sdf::SDF frameModel = getFrameSdfModel(taskFrameName, frameTransparency);
        addSdfToWorld(frameModel);

        sdf::SDF targetModel = getFrameSdfModel(taskTargetName, targetTransparency);
        addSdfToWorld(targetModel);
        taskFramesActiveInWorld.push_back(taskName);

        message = "Added task frames for " + taskName;
        worked = true;
    }
}

void OcraGuiPlugin::removeTaskFrames(const std::string& taskName, bool& worked, std::string& message)
{
    if (checkIfTaskFramesExist(taskName)) {
        std::string taskFrameName = taskName + "-Frame";
        std::string taskTargetName = taskName + "-Target";
        this->world->RemoveModel(taskFrameName);
        this->world->RemoveModel(taskTargetName);
        removeTaskNameFromList(taskName);
        message = "Removed task frames for " + taskName;
        worked = true;
    } else {
        message = "Task frames for " + taskName + " do not exist.";
        worked = false;
    }
}

void OcraGuiPlugin::removeTaskNameFromList(const std::string& taskName)
{
    for (int i=0; i<taskFramesActiveInWorld.size(); ++i) {
        if (taskFramesActiveInWorld[i]==taskName) {
            taskFramesActiveInWorld.erase(taskFramesActiveInWorld.begin()+i);
        }
    }
}


bool OcraGuiPlugin::checkIfTaskFramesExist(const std::string& taskName)
{
    for (auto name : taskFramesActiveInWorld) {
        if (name == taskName) {
            return true;
        }
    }
    return false;
}

sdf::SDF OcraGuiPlugin::getFrameSdfModel(const std::string& frameName, double transparencyValue)
{
    sdf::SDF frameSDF;
    frameSDF.SetFromString(
        "<?xml version='1.0'?>\
        <sdf version='1.4'>\
        <model name='"+frameName+"'>\
            <pose>0 0 0 0 0 0</pose>\
            <static>false</static>\
            <link name='origin'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <box>\
                              <size>0.02 0.02 0.02</size>\
                          </box>\
                    </geometry>\
                    <material>\
                        <ambient>1 1 1 1</ambient>\
                        <diffuse>1 1 1 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='x_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.03 0.0 0.0 0.0 1.57 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>1 0 0 1</ambient>\
                        <diffuse>1 0 0 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='y_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.0 0.03 0.0 1.57 0.0 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>0 1 0 1</ambient>\
                        <diffuse>0 1 0 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <link name='z_axis'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <pose>0.0 0.0 0.03 0.0 0.0 0.0</pose>\
                    <transparency>"+std::to_string(transparencyValue)+"</transparency>\
                    <geometry>\
                          <cylinder>\
                              <length>0.04</length>\
                              <radius>0.005</radius>\
                          </cylinder>\
                    </geometry>\
                    <material>\
                        <ambient>0 0 1 1</ambient>\
                        <diffuse>0 0 1 1</diffuse>\
                        <specular>0.1 0.1 0.1 1</specular>\
                        <emissive>0 0 0 0</emissive>\
                    </material>\
                </visual>\
            </link>\
            <plugin name='yarp_model_move' filename='libyarp_model_move.so'/>\
        </model>\
        </sdf>"
    );
    return frameSDF;
}
