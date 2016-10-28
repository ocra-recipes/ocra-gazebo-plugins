#ifndef _OCRA_TASK_WIDGET_HH_
#define _OCRA_TASK_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ocra-recipes/ClientCommunications.h>
#include "TaskConnectionRelay.hh"

#endif
#include <QTimer>
#include <QButtonGroup>
#include <sstream>
#include <gazebo/msgs/msgs.hh>


namespace gazebo
{
class GAZEBO_VISIBLE OcraTaskWidget : public GUIPlugin
{
    Q_OBJECT
    using TaskRelayMap = std::map<std::string, std::shared_ptr<TaskConnectionRelay>>;
    public:
      OcraTaskWidget();
      virtual ~OcraTaskWidget();

    public slots:
        void OnButton();
        void addTaskFrames(int taskIndex);
        void reconnectRelays();


    private:
        void initializeGui();
        void sendModelMsgToGazebo(const sdf::SDF& sdfModel);
        sdf::SDF getFrameSdfModel(const std::string& frameName, double transparencyValue);
        void showUserInformation(const std::string& message, int timeout=5000); // in ms
        void showTaskList();
        void hideTaskList();
        void addTaskFrames(const std::string& taskName);
        void getTaskList();





    private:
        // Node used to establish communication with gzserver.
        transport::NodePtr node;
        // Publisher of factory messages.
        transport::PublisherPtr factoryPub;

        // ocra_recipes::ClientCommunications clientComs;
        TaskRelayMap taskRelayMap;
        std::vector<std::string> taskNames;
        std::vector<bool> taskActivationVector;

        //GUI Related:
        QHBoxLayout *topLayout;
        QLabel *informUserLabel;
        QButtonGroup* taskButtons;
        // QFrame* buttonFrame;
        QVBoxLayout *buttonGroupLayout;
        bool tasksDisplayed;


};
} // namespace gazebo
#endif //_OCRA_TASK_WIDGET_HH_
