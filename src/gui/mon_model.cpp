// Qt model for a rosmon instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "mon_model.h"

#include <QColor>

#include <ros/node_handle.h>

Q_DECLARE_METATYPE(rosmon::StateConstPtr)

namespace rosmon
{

MonModel::MonModel(ros::NodeHandle& nh, QObject* parent)
 : QAbstractTableModel(parent)
 , m_nh(nh)
 , m_namespace("/rosmon")
{
	qRegisterMetaType<rosmon::StateConstPtr>();
	connect(this, SIGNAL(stateReceived(rosmon::StateConstPtr)),
		SLOT(updateState(rosmon::StateConstPtr)),
		Qt::QueuedConnection
	);
}

MonModel::~MonModel()
{
}

void MonModel::setNamespace(const QString& ns)
{
	m_namespace = ns;

	try
	{
		ros::NodeHandle nh(m_nh, ns.toStdString());
		m_sub_state = nh.subscribe("state", 1, &MonModel::stateReceived, this);
	}
	catch(ros::InvalidNameException& name)
	{
		ROS_WARN("Got invalid name '%s'", qPrintable(ns));
		m_sub_state.shutdown();
	}

	beginResetModel();
	m_entries.clear();
	endResetModel();
}

void MonModel::unsubscribe()
{
	m_sub_state.shutdown();
}

void MonModel::updateState(const rosmon::StateConstPtr& state)
{
	std::vector<bool> covered(m_entries.size(), false);

	// Look for changed & new rows
	for(auto nodeState : state->nodes)
	{
		Entry key;
		key.name = QString::fromStdString(nodeState.name);
		key.state = nodeState.state;
		key.restartCount = nodeState.restartCount;

		auto it = std::lower_bound(m_entries.begin(), m_entries.end(), key);
		int row = it - m_entries.begin();

		if(it == m_entries.end() || it->name != key.name)
		{
			printf("insert %d\n", row);
			beginInsertRows(QModelIndex(), row, row);
			m_entries.insert(it, key);
			covered.insert(covered.begin() + row, true);
			endInsertRows();
		}
		else
		{
			*it = key;
			covered[row] = true;
			dataChanged(index(row, 0), index(row, COL_COUNT));
		}
	}

	// Look for deleted rows
	auto it = m_entries.begin();
	std::size_t covIdx = 0;

	while(it != m_entries.end())
	{
		int row = it - m_entries.begin();

		if(!covered[covIdx])
		{
			beginRemoveRows(QModelIndex(), row, row);
			it = m_entries.erase(it);
			endRemoveRows();
		}
		else
			++it;

		++covIdx;
	}
}

int MonModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_entries.size();
}

int MonModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return COL_COUNT;
}

QVariant MonModel::data(const QModelIndex& index, int role) const
{
	if(!index.isValid())
		return QVariant();

	if(index.row() < 0 || index.row() >= (int)m_entries.size())
		return QVariant();

	const Entry& entry = m_entries[index.row()];

	switch(role)
	{
		case Qt::DisplayRole:
			switch(index.column())
			{
				case COL_NAME:
					return entry.name;
				case COL_RESTART_COUNT:
					return entry.restartCount;
			}
			break;
		case Qt::BackgroundColorRole:
			switch(entry.state)
			{
				case rosmon::NodeState::IDLE:
					return QVariant();
				case rosmon::NodeState::RUNNING:
					return QColor(200, 255, 200);
				case rosmon::NodeState::CRASHED:
					return QColor(255, 100, 100);
				case rosmon::NodeState::WAITING:
					return QColor(255, 255, 128);
			}
			break;
	}

	return QVariant();
}

QVariant MonModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(role != Qt::DisplayRole || orientation != Qt::Horizontal)
		return QAbstractTableModel::headerData(section, orientation, role);

	switch(section)
	{
		case COL_NAME:          return "Node";
		case COL_RESTART_COUNT: return "#Restarts";
	}

	return QVariant();
}

QString MonModel::nodeName(int row) const
{
	if(row < 0 || row > (int)m_entries.size())
		return QString();

	return m_entries[row].name;
}

}

