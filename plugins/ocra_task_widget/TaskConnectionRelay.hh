#include <yarp/os/all.h>
#include <ocra-recipes/TaskConnection.h>
#include <ocra/control/TaskState.h>

#ifndef _TASK_CONNECTION_RELAY_HH_
#define _TASK_CONNECTION_RELAY_HH_

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

    yarp::os::Port framePortOut;
    yarp::os::Port targetPortOut;
    yarp::os::Port taskStateInPort;

    ocra::TaskState currentFrameState;
    ocra::TaskState currentTargetState;

    Eigen::VectorXd framePosition;
    Eigen::VectorXd targetPosition;

    yarp::os::Bottle frameBottle;
    yarp::os::Bottle targetBottle;


private:
    void sendPosesToGazebo();


public:
    TaskConnectionRelay(const std::string& _taskName);
    ~TaskConnectionRelay();
    void parseInput(yarp::os::Bottle& input);


    /************** controlInputCallback *************/
    class InputCallback : public yarp::os::PortReader {
        private:
            TaskConnectionRelay& tcRef;

        public:
            InputCallback(TaskConnectionRelay& _tcRef);

            virtual bool read(yarp::os::ConnectionReader& connection);
    };
    /************** controlInputCallback *************/


private:
    std::shared_ptr<InputCallback> inputCallback;

};

#endif //_TASK_CONNECTION_RELAY_HH_
