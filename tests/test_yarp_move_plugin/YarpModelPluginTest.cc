#include <iostream>
#include "yarp/os/Port.h"
#include "yarp/os/Bottle.h"
#include "yarp/os/Network.h"
#include "yarp/os/Time.h"

int main(int argc, char const *argv[]) {

    yarp::os::Network yarp;
    yarp::os::Port outputPort;
    std::string portName("/test/yarp_model_move:o");
    outputPort.open(portName);

    yarp.connect(portName, "/ComTask:i");

    yarp::os::Bottle bottle;
    double stepSize = 0.01; //1cm

    double z = 0.0;
    double roll = 0.0;

    while (z <= 1.0) {
        bottle.clear();
        bottle.addDouble(0.0);
        bottle.addDouble(0.0);
        bottle.addDouble(z);
        outputPort.write(bottle);
        std::cout << "Sent z = " << z << std::endl;
        yarp::os::Time::delay(0.01);
        z += stepSize;
    }

    while (roll <= 3.14) {
        bottle.clear();
        bottle.addDouble(0.0);
        bottle.addDouble(0.0);
        bottle.addDouble(z);
        bottle.addDouble(roll);
        bottle.addDouble(roll);
        bottle.addDouble(0.0);
        outputPort.write(bottle);
        std::cout << "Sent roll = " << roll << std::endl;
        yarp::os::Time::delay(0.01);
        roll += stepSize;
    }



    return 0;
}
