// rqt GUI for rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MON_GUI_H
#define MON_GUI_H

#include <rqt_gui_cpp/plugin.h>

#include "ui_mon_gui.h"

#include "node_model.h"
#include "rosmon_model.h"

namespace rosmon
{

class MonGUI : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	MonGUI() = default;
	~MonGUI() override = default;

	void initPlugin(qt_gui_cpp::PluginContext &ctx) override;
	void shutdownPlugin() override;

	void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
	void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;
private Q_SLOTS:
	void showContextMenu(const QPoint& point);
	void setNamespace(const QString& ns);
	void checkAutoTopic();
private:
	Ui_MonGUI m_ui;
	QWidget* m_w;
	NodeModel* m_model;
	ROSMonModel* m_rosmonModel;
	QTimer* m_autoTimer;
};

}

#endif
