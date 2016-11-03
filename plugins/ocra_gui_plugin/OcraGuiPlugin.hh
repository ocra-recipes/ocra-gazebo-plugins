#ifndef OCRA_GUI_PLUGIN_HH
#define OCRA_GUI_PLUGIN_HH



#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{

class OcraGuiPlugin : public WorldPlugin
{
public:

    OcraGuiPlugin();
    virtual ~OcraGuiPlugin();
    void Load(physics::WorldPtr _parent, sdf::ElementPtr);

    void parseInputAndReply(const yarp::os::Bottle& in, yarp::os::Bottle& out);


    class RpcServerCallback : public yarp::os::PortReader
    {
    private:
        OcraGuiPlugin* plugin;

    public:
        RpcServerCallback(){}
        RpcServerCallback(OcraGuiPlugin* pluginPtr) : plugin(pluginPtr) {}
        virtual ~RpcServerCallback()
        {
            plugin = NULL;
        }

        virtual bool read(yarp::os::ConnectionReader& connection)
        {
            yarp::os::Bottle in, out;
            bool ok = in.read(connection);
            if (!ok) {
                return false;
            } else {
                plugin->parseInputAndReply(in, out);
            }
            yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
            if (returnToSender!=NULL) {
                out.write(*returnToSender);
            }
            return true;
        }
    };

private:
    void addSdfToWorld(const sdf::SDF& sphereSDF);
    void addTaskFrames(const std::string& taskName);
    void removeTaskFrames(const std::string& taskName);
    bool checkIfTaskFramesExist(const std::string& taskName);
    void sendModelMsgToGazebo(const sdf::SDF& sdfModel);
    sdf::SDF getFrameSdfModel(const std::string& frameName, double transparencyValue);

private:
    physics::WorldPtr world;
    yarp::os::Network yarp;
    yarp::os::RpcServer port;
    std::vector<std::string> taskFramesActiveInWorld;
    RpcServerCallback callback;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OcraGuiPlugin)

} // namespace gazebo

#endif //OCRA_GUI_PLUGIN_HH
