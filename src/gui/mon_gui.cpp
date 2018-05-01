// rqt GUI for rosmon
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "mon_gui.h"

#if HAVE_PLUGINLIB_NEW_HEADERS
#include <pluginlib/class_list_macros.hpp>
#else
#include <pluginlib/class_list_macros.h>
#endif

#include <QMenu>
#include <QMessageBox>
#include <QTimer>
#include <QDebug>
#include <QSortFilterProxyModel>

#include <rosmon/StartStop.h>

#include <ros/service.h>

#include <thread>

#include "bar_delegate.h"

namespace rosmon
{

void MonGUI::initPlugin(qt_gui_cpp::PluginContext& ctx)
{
	m_w = new QWidget;
	m_ui.setupUi(m_w);

	m_rosmonModel = new ROSMonModel(this);
	m_ui.nodeBox->setModel(m_rosmonModel);

	m_model = new NodeModel(getNodeHandle(), this);

#if QT_VERSION_MAJOR >= 5
	m_ui.nodeBox->setCurrentText("[auto]");
#else
	m_ui.nodeBox->setEditText("[auto]");
#endif

	connect(m_ui.nodeBox, SIGNAL(editTextChanged(QString)),
		SLOT(setNamespace(QString))
	);

	auto sortFilterProxy = new QSortFilterProxyModel(this);
	sortFilterProxy->setSourceModel(m_model);
	sortFilterProxy->setDynamicSortFilter(true);
	m_ui.tableView->setModel(sortFilterProxy);

	m_ui.tableView->setSortingEnabled(true);
	m_ui.tableView->setSelectionBehavior(QAbstractItemView::SelectRows);
	m_ui.tableView->setSelectionMode(QAbstractItemView::ExtendedSelection);
	m_ui.tableView->setContextMenuPolicy(Qt::CustomContextMenu);

	// Display colored bar graph for CPU load
	{
		auto loadDelegate = new BarDelegate(m_ui.tableView);
		int numCores = std::thread::hardware_concurrency();
		if(numCores <= 0)
			numCores = 1;

		loadDelegate->setRange(0.0, numCores);
		m_ui.tableView->setItemDelegateForColumn(NodeModel::COL_LOAD,
			loadDelegate
		);
	}

	// Display colored bar graph for memory consumption
	{
		auto memDelegate = new BarDelegate(m_ui.tableView);

		long int av_pages = sysconf(_SC_PHYS_PAGES);
		long int page_size = sysconf(_SC_PAGESIZE);
		int64_t availableMemory = av_pages * page_size;
		if(availableMemory < 0)
			availableMemory = 1;

		memDelegate->setRange(0.0, availableMemory);
		m_ui.tableView->setItemDelegateForColumn(NodeModel::COL_MEMORY,
			memDelegate
		);
	}

	connect(m_ui.tableView, SIGNAL(customContextMenuRequested(QPoint)),
		SLOT(showContextMenu(QPoint))
	);
	connect(m_model, SIGNAL(stateReceived(rosmon::StateConstPtr)),
		m_ui.tableView, SLOT(resizeRowsToContents())
	);

	m_autoTimer = new QTimer(this);
	m_autoTimer->setInterval(1000);
	connect(m_autoTimer, SIGNAL(timeout()), SLOT(checkAutoTopic()));

	ctx.addWidget(m_w);
	setNamespace("[auto]");
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
		m_ui.nodeBox->setEditText(ns);
		setNamespace(ns);
	}

	if(instanceSettings.contains("viewState"))
		m_ui.tableView->horizontalHeader()->restoreState(instanceSettings.value("viewState").toByteArray());
}

void MonGUI::saveSettings(qt_gui_cpp::Settings& pluginSettings, qt_gui_cpp::Settings& instanceSettings) const
{
	instanceSettings.setValue("namespace", m_ui.nodeBox->currentText());
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

		if(!ros::service::call(m_model->namespaceString().toStdString() + "/start_stop", srv))
			QMessageBox::critical(m_w, "Failure", "Could not call start_stop service");
	}
}

void MonGUI::checkAutoTopic()
{
	// Stupid heuristic: Select the first matching
	if(m_rosmonModel->rowCount(QModelIndex()) > 1)
	{
		QString ns = m_rosmonModel->data(m_rosmonModel->index(1), Qt::DisplayRole).toString();

		if(m_model->namespaceString() != ns)
			m_model->setNamespace(ns);
	}
	else
		m_model->setNamespace("");
}

void MonGUI::setNamespace(const QString& ns)
{
	if(ns == "[auto]")
	{
		checkAutoTopic();
		m_autoTimer->start();
	}
	else
	{
		m_model->setNamespace(ns);
		m_autoTimer->stop();
	}
}

}

PLUGINLIB_EXPORT_CLASS(rosmon::MonGUI, rqt_gui_cpp::Plugin)
