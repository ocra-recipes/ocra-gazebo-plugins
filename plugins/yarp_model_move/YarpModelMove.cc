#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <yarp/os/all.h>
#include <iostream>

namespace gazebo
{

class YarpModelMove : public ModelPlugin
{
public:
    yarp::os::Network yarp;

    YarpModelMove()
    {
    }

    ~YarpModelMove()
    {
        port.close();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;
        port.open("/Gazebo/"+this->model->GetName()+":i");
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&YarpModelMove::OnUpdate, this, _1));
    }

    void OnUpdate(const common::UpdateInfo & )
    {
        b = port.read(false);
        if (b!=NULL)
        {
            // std::cout << "b: " << b->toString() << std::endl;

            if(b->size()==3) {
                this->model->SetWorldPose(math::Pose(   b->get(0).asDouble(),
                                                        b->get(1).asDouble(),
                                                        b->get(2).asDouble(),
                                                        0.0,
                                                        0.0,
                                                        0.0));
            } else if (b->size()==6) {
                this->model->SetWorldPose(math::Pose(   b->get(0).asDouble(),
                                                        b->get(1).asDouble(),
                                                        b->get(2).asDouble(),
                                                        b->get(3).asDouble(),
                                                        b->get(4).asDouble(),
                                                        b->get(5).asDouble()));
            }
        }
    }

private:
    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    yarp::os::BufferedPort<yarp::os::Bottle> port;
    yarp::os::Bottle *b;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(YarpModelMove)
}
