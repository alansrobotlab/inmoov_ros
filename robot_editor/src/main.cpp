#include <QApplication>
#include <ros/ros.h>
#include "robot_editor.h"
#include <ui_main_window.h>

int main(int argc, char** argv)
{
	if(!ros::isInitialized())
	{
		ros::init(argc, argv, "robot_editor", ros::init_options::AnonymousName);
	}

	QApplication app(argc, argv);

	RobotEditor robot_editor;
	robot_editor.show();

	return app.exec();
}
