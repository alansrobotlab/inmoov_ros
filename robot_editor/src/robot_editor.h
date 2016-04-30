#ifndef ROBOT_EDITOR_ROBOT_EDITOR_H_
#define ROBOT_EDITOR_ROBOT_EDITOR_H_

#include <QObject>
#include <ui_main_window.h>

#include <ros/ros.h>
#include <string>
#include <map>

#include <boost/thread/mutex.hpp>


class QMainWindow;
class RobotPreview;
namespace robot_state_publisher { class RobotStatePublisher; }
namespace KDL { class Tree; }
namespace boost { class thread; }

class RobotEditor : public QObject
{
Q_OBJECT
public:
	RobotEditor();
	~RobotEditor();

    void show();

public Q_SLOTS:
	void openTrigger();
	void saveTrigger();
	void saveAsTrigger();
	void exitTrigger();

private:
	void updateURDF(const std::string& urdf);
	void publishJointStates();
	
private:
    QMainWindow main_window_;
	Ui::MainWindow main_window_ui_;
	RobotPreview* robot_preview_;

	QString file_name_;

	ros::NodeHandle nh_;

	boost::mutex state_pub_mutex_;
	KDL::Tree* robot_tree_ = NULL;
	robot_state_publisher::RobotStatePublisher* robot_state_pub_ = NULL;
	boost::thread* publisher_thread_;
	std::map<std::string, double> joint_positions_;
};

#endif
