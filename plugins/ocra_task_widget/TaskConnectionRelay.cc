#include "TaskConnectionRelay.hh"

TaskConnectionRelay::TaskConnectionRelay(const std::string& _taskName)
: taskName(_taskName)
{
    taskCon = std::make_shared<ocra_recipes::TaskConnection>(taskName);
    taskCon->openControlPorts();
    taskType = taskCon->getTaskTypeAsString();
    // taskStatePortOutName = taskCon->getTaskOutputPortName();
    yarp::os::Bottle reply;
    taskCon->queryTask(ocra::TASK_MESSAGE::GET_CONTROL_PORT_NAMES, reply);
    if(reply.size() == 3) {
        taskStatePortOutName = reply.get(1).asString();
        taskDesiredStatePortOutName = reply.get(2).asString();
    }

    framePortOutName = "/TaskConnectionRelay/"+taskName+"-Frame"+":o";
    targetPortOutName = "/TaskConnectionRelay/"+taskName+"-Target"+":o";
    framePortInName = "/Gazebo/"+taskName+"-Frame"+":i";
    targetPortInName = "/Gazebo/"+taskName+"-Target"+":i";

    taskStatePortInName = "/TaskConnectionRelay/"+taskName+"/state:i";
    taskDesiredStatePortInName = "/TaskConnectionRelay/"+taskName+"/desired_state:i";

    framePortOut = std::make_shared<yarp::os::Port>();
    targetPortOut = std::make_shared<yarp::os::Port>();


    framePortOut->open(framePortOutName);
    targetPortOut->open(targetPortOutName);
    taskStateInPort.open(taskStatePortInName);
    taskDesiredStateInPort.open(taskDesiredStatePortInName);


    connect();

    inputStateCallback = std::make_shared<RelayCallback>(framePortOut);
    taskStateInPort.setReader(*inputStateCallback);

    inputDesiredStateCallback = std::make_shared<RelayCallback>(targetPortOut);
    taskDesiredStateInPort.setReader(*inputDesiredStateCallback);
}

TaskConnectionRelay::~TaskConnectionRelay()
{
    framePortOut->close();
    targetPortOut->close();
    taskStateInPort.close();
    taskDesiredStateInPort.close();
}

void TaskConnectionRelay::connect()
{
    taskCon->reconnect();
    taskCon->openControlPorts();
    while ( !yarp.connect(framePortOutName, framePortInName) ){yarp::os::Time::delay(0.01);}
    while ( !yarp.connect(targetPortOutName, targetPortInName) ){yarp::os::Time::delay(0.01);}
    while ( !yarp.connect(taskStatePortOutName, taskStatePortInName) ){yarp::os::Time::delay(0.01);}
    while ( !yarp.connect(taskDesiredStatePortOutName, taskDesiredStatePortInName) ){yarp::os::Time::delay(0.01);}
}

/**************************************************************************************************
                                    PortReader Class
**************************************************************************************************/
RelayCallback::RelayCallback(std::shared_ptr<yarp::os::Port> _relayPort)
: relayPort(_relayPort)
{
    //do nothing
}

RelayCallback::~RelayCallback()
{
    // delete relayPort;
}

bool RelayCallback::read(yarp::os::ConnectionReader& connection)
{
    inputBottle.clear();
    if (inputBottle.read(connection)){
        state.extractFromBottle(inputBottle, dummyInt);
        sendPoseToGazebo();
        return true;
    }
    else{
        return false;
    }
}

void RelayCallback::sendPoseToGazebo()
{
    outputBottle.clear();

    displacementToXYZRPY(state.getPosition(), position);

    for (auto i=0; i<position.size(); ++i) {
        outputBottle.addDouble(position[i]);
    }
    // std::cout << "outputBottle: " << outputBottle.toString() << std::endl;
    relayPort->write(outputBottle);
    // return;
}
/**************************************************************************************************
**************************************************************************************************/
