// rqt GUI for rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "mon_gui.h"

#include <pluginlib/class_list_macros.h>

#include <QMenu>
#include <QMessageBox>

#include <rosmon/StartStop.h>

#include <ros/service.h>

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

	m_ui.tableView->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_ui.tableView, SIGNAL(customContextMenuRequested(QPoint)),
		SLOT(showContextMenu(QPoint))
	);
	connect(m_model, SIGNAL(stateReceived(rosmon::StateConstPtr)),
		m_ui.tableView, SLOT(resizeRowsToContents())
	);

	ctx.addWidget(m_w);
}

void MonGUI::shutdownPlugin()
{
	m_model->unsubscribe();
}

void MonGUI::restoreSettings(const qt_gui_cpp::Settings& pluginSettings, const qt_gui_cpp::Settings& instanceSettings)
{
	if(instanceSettings.contains("namespace"))
	{
		QString ns = instanceSettings.value("namespace").toString();
		m_ui.nodeEdit->setText(ns);
	}

	if(instanceSettings.contains("viewState"))
		m_ui.tableView->horizontalHeader()->restoreState(instanceSettings.value("viewState").toByteArray());
}

void MonGUI::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("namespace", m_ui.nodeEdit->text());
	instanceSettings.setValue("viewState", m_ui.tableView->horizontalHeader()->saveState());
}

void MonGUI::showContextMenu(const QPoint& point)
{
	QModelIndex index = m_ui.tableView->indexAt(point);

	if(!index.isValid())
		return;

	QMenu menu(m_ui.tableView);

	QAction* startAction = menu.addAction("Start");
	startAction->setProperty("action", rosmon::StartStopRequest::START);

	QAction* stopAction = menu.addAction("Stop");
	stopAction->setProperty("action", rosmon::StartStopRequest::STOP);

	QAction* restartAction = menu.addAction("Restart");
	restartAction->setProperty("action", rosmon::StartStopRequest::RESTART);

	QAction* triggered = menu.exec(m_ui.tableView->viewport()->mapToGlobal(point));

	if(triggered)
	{
		rosmon::StartStop srv;
		srv.request.node = m_model->nodeName(index.row()).toStdString();
		srv.request.action = triggered->property("action").toInt();

		if(!ros::service::call(m_ui.nodeEdit->text().toStdString() + "/start_stop", srv))
			QMessageBox::critical(m_w, "Failure", "Could not call start_stop service");
	}
}

}

PLUGINLIB_EXPORT_CLASS(rosmon::MonGUI, rqt_gui_cpp::Plugin)
