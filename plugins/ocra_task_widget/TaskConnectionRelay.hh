#include <yarp/os/all.h>
#include <ocra-recipes/TaskConnection.h>
#include <ocra/control/TaskState.h>
#include <Eigen/Lgsm>
#include <cmath>

#ifndef _TASK_CONNECTION_RELAY_HH_
#define _TASK_CONNECTION_RELAY_HH_





inline void quaternionToRPY(const Eigen::Rotation3d& q, double& roll, double& pitch, double& yaw)
{
	// double ysqr = q.y() * q.y();
	// double t0 = -2.0 * (ysqr + q.z() * q.z()) + 1.0;
	// double t1 = +2.0 * (q.x() * q.y() - q.w() * q.z());
	// double t2 = -2.0 * (q.x() * q.z() + q.w() * q.y());
	// double t3 = +2.0 * (q.y() * q.z() - q.w() * q.x());
	// double t4 = -2.0 * (q.x() * q.x() + ysqr) + 1.0;
    //
	// t2 = t2 > 1.0 ? 1.0 : t2;
	// t2 = t2 < -1.0 ? -1.0 : t2;
    //
    // roll = std::atan2(t3, t4);
	// pitch = std::asin(t2);
	// yaw = std::atan2(t1, t0);

    yaw = std::atan( 2.0*( q.x()*q.y() + q.w()*q.z() ) / ( q.w()*q.w() - q.z()*q.z() - q.y()*q.y() + q.x()*q.x() ) );

    pitch = std::asin( -2.0*( q.x()*q.z()-q.y()*q.w() ) );

    roll = std::atan( 2.0*( q.y()*q.z() + q.x()*q.w() ) / ( q.w()*q.w() + q.z()*q.z() - q.y()*q.y() - q.x()*q.x() ) );
}


inline void displacementToXYZRPY(const Eigen::Displacementd& disp, std::vector<double>& v)
{
    v.resize(6);
    v[0] = disp.x();
    v[1] = disp.y() + 0.068;
    v[2] = disp.z();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToRPY(disp.getRotation(), roll, pitch, yaw);
    v[3] = roll;
    v[4] = pitch;
    v[5] = yaw;
}


class RelayCallback : public yarp::os::PortReader
{
private:
    ocra::TaskState state;
    std::shared_ptr<yarp::os::Port> relayPort;
    yarp::os::Bottle inputBottle;
    yarp::os::Bottle outputBottle;
    int dummyInt;
    std::vector<double> position;


public:
    RelayCallback(std::shared_ptr<yarp::os::Port> _relayPort);
    virtual ~RelayCallback();
    virtual bool read(yarp::os::ConnectionReader& connection);
    void sendPoseToGazebo();

};

class TaskConnectionRelay {
private:
    yarp::os::Network yarp;

    std::string taskName;
    std::string taskType;

    std::string taskStatePortOutName;
    std::string taskStatePortInName;

    std::string framePortInName;
    std::string framePortOutName;

    std::string targetPortInName;
    std::string targetPortOutName;

    ocra_recipes::TaskConnection::Ptr taskCon;

    std::shared_ptr<yarp::os::Port> framePortOut;
    std::shared_ptr<yarp::os::Port> targetPortOut;
    yarp::os::Port taskStateInPort;

    std::string taskDesiredStatePortOutName;
    std::string taskDesiredStatePortInName;
    yarp::os::Port taskDesiredStateInPort;

    // ocra::TaskState currentFrameState;
    // ocra::TaskState currentTargetState;
    //
    // std::vector<double> framePosition;
    // std::vector<double> targetPosition;
    //
    // yarp::os::Bottle frameBottle;
    // yarp::os::Bottle targetBottle;

private:
    // void sendPosesToGazebo();


public:
    TaskConnectionRelay(const std::string& _taskName);
    ~TaskConnectionRelay();
    // void parseInput(yarp::os::Bottle& input);





private:
    std::shared_ptr<RelayCallback> inputStateCallback;
    std::shared_ptr<RelayCallback> inputDesiredStateCallback;

};

#endif //_TASK_CONNECTION_RELAY_HH_
