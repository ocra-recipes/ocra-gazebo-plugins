#include "TaskConnectionRelay.hh"

TaskConnectionRelay::TaskConnectionRelay(const std::string& _taskName)
: taskName(_taskName)
{
    taskCon = std::make_shared<ocra_recipes::TaskConnection>(taskName);
    taskCon->openControlPorts();
    taskType = taskCon->getTaskTypeAsString();
    taskStatePortOutName = taskCon->getTaskOutputPortName();

    framePortInName = "/"+taskName+"-Frame"+":i";
    framePortOutName = "/"+taskName+"-Frame"+":o";
    targetPortInName = "/"+taskName+"-Target"+":i";
    targetPortOutName = "/"+taskName+"-Target"+":o";

    taskStatePortInName = "/"+taskName+"/state:i";

    framePortOut.open(framePortOutName);
    targetPortOut.open(targetPortOutName);
    taskStateInPort.open(taskStatePortInName);

    yarp.connect(framePortOutName, framePortInName);
    yarp.connect(targetPortOutName, targetPortInName);

    inputCallback = std::make_shared<InputCallback>(*this);
    taskStateInPort.setReader(*inputCallback);
    yarp.connect(taskStatePortOutName, taskStatePortInName);
}

TaskConnectionRelay::~TaskConnectionRelay()
{
    framePortOut.close();
    targetPortOut.close();
}

void TaskConnectionRelay::sendPosesToGazebo()
{
    framePosition = currentFrameState.getPosition().getTranslation();
    targetPosition = currentTargetState.getPosition().getTranslation();
    for (int i=0; i<3; ++i) {
        frameBottle.addDouble(framePosition(i));
        targetBottle.addDouble(targetPosition(i));
    }
    framePortOut.write(frameBottle);
    targetPortOut.write(targetBottle);
}

void TaskConnectionRelay::parseInput(yarp::os::Bottle& input)
{
    int dummy;
    currentFrameState.extractFromBottle(input, dummy);
    currentTargetState = taskCon->getDesiredTaskState();
    sendPosesToGazebo();
}

/**************************************************************************************************
                                    Nested PortReader Class
**************************************************************************************************/
TaskConnectionRelay::InputCallback::InputCallback(TaskConnectionRelay& _tcRef)
: tcRef(_tcRef)
{
    //do nothing
}

bool TaskConnectionRelay::InputCallback::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle input;
    if (input.read(connection)){
        tcRef.parseInput(input);
        return true;
    }
    else{
        return false;
    }
}
/**************************************************************************************************
**************************************************************************************************/
