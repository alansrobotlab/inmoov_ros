#ifndef ROBOT_EDITOR_ROBOT_PREVIEW_H_
#define ROBOT_EDITOR_ROBOT_PREVIEW_H_

#include <QWidget>

namespace rviz
{
	class Display;
	class RenderPanel;
	class VisualizationManager;
}

class RobotPreview: public QWidget
{
	Q_OBJECT
public:
	RobotPreview(QWidget* parent=0);
	virtual ~RobotPreview();

	void refresh(const std::string& fixed_frame = "/map");

private:
	rviz::VisualizationManager* manager_;
	rviz::RenderPanel* render_panel_;
	rviz::Display* grid_;
	rviz::Display* robot_model_ = NULL;
};

#endif
