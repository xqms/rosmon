// rqt GUI for rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "mon_gui.h"

#include <pluginlib/class_list_macros.h>

namespace rosmon
{

MonGUI::MonGUI()
{
}

MonGUI::~MonGUI()
{
}

void MonGUI::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_w = new QWidget;
	m_ui.setupUi(m_w);

	m_model = new MonModel(getNodeHandle(), this);

	connect(m_ui.nodeEdit, SIGNAL(textChanged(QString)),
		m_model, SLOT(setNamespace(QString))
	);

	m_ui.tableView->setModel(m_model);

	ctx.addWidget(m_w);
}

void MonGUI::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("namespace"))
	{
		QString ns = instanceSettings.value("namespace").toString();
		m_ui.nodeEdit->setText(ns);
	}
}

void MonGUI::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("namespace", m_ui.nodeEdit->text());
}

}

PLUGINLIB_EXPORT_CLASS(rosmon::MonGUI, rqt_gui_cpp::Plugin)
