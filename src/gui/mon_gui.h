// rqt GUI for rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef MON_GUI_H
#define MON_GUI_H

#include <rqt_gui_cpp/plugin.h>

#include "ui_mon_gui.h"

#include "mon_model.h"

namespace rosmon
{

class MonGUI : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	MonGUI();
	virtual ~MonGUI();

	virtual void initPlugin(qt_gui_cpp::PluginContext &ctx) override;
	virtual void shutdownPlugin() override;

	virtual void saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const override;
	virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings) override;
private Q_SLOTS:
	void updateTopic();
	void showContextMenu(const QPoint& point);
private:
	Ui_MonGUI m_ui;
	QWidget* m_w;
	MonModel* m_model;
};

}

#endif
