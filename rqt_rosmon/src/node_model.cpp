// Qt model for nodes controlled by a rosmon instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "node_model.h"

#include <QColor>
#include <QBrush>

#include <ros/node_handle.h>

#include "format_data_size.h"

Q_DECLARE_METATYPE(rosmon_msgs::StateConstPtr)

namespace rqt_rosmon
{

NodeModel::NodeModel(ros::NodeHandle& nh, QObject* parent)
 : QAbstractTableModel(parent)
 , m_nh(nh)
 , m_namespace("/rosmon")
{
	qRegisterMetaType<rosmon_msgs::StateConstPtr>();
	connect(this, SIGNAL(stateReceived(rosmon_msgs::StateConstPtr)),
		SLOT(updateState(rosmon_msgs::StateConstPtr)),
		Qt::QueuedConnection
	);
}

void NodeModel::setNamespace(const QString& ns)
{
	m_namespace = ns;

	if(!ns.isEmpty())
	{
		try
		{
			ros::NodeHandle nh(m_nh, ns.toStdString());
			m_sub_state = nh.subscribe("state", 1, &NodeModel::stateReceived, this);
		}
		catch(ros::InvalidNameException& name)
		{
			ROS_WARN("Got invalid name '%s'", qPrintable(ns));
			m_sub_state.shutdown();
		}
	}
	else
		m_sub_state.shutdown();

	beginResetModel();
	m_entries.clear();
	endResetModel();
}

void NodeModel::unsubscribe()
{
	m_sub_state.shutdown();
}

void NodeModel::updateState(const rosmon_msgs::StateConstPtr& state)
{
	std::vector<bool> covered(m_entries.size(), false);

	// Look for changed & new rows
	for(const auto& nodeState : state->nodes)
	{
		Entry key;
		key.name = QString::fromStdString(nodeState.name);
		key.ns = QString::fromStdString(nodeState.ns);
		key.state = nodeState.state;
		key.restartCount = nodeState.restart_count;
		key.load = nodeState.system_load + nodeState.user_load;
		key.memory = nodeState.memory;

		auto it = std::lower_bound(m_entries.begin(), m_entries.end(), key);
		int row = it - m_entries.begin();

		if(it == m_entries.end() || !((it->name == key.name) && (it->ns == key.ns)))
		{
			beginInsertRows(QModelIndex(), row, row);
			m_entries.insert(it, key);
			covered.insert(covered.begin() + row, true);
			endInsertRows();
		}
		else
		{
			*it = key;
			covered[row] = true;
			dataChanged(index(row, 0), index(row, COL_COUNT-1));
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

int NodeModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_entries.size();
}

int NodeModel::columnCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return COL_COUNT;
}

QVariant NodeModel::data(const QModelIndex& index, int role) const
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
				case COL_NAMESPACE:
					return entry.ns;
				case COL_LOAD:
					return QString::number(entry.load, 'f', 2);
				case COL_MEMORY:
					return formattedDataSize(entry.memory, 2);
				case COL_RESTART_COUNT:
					return entry.restartCount;
			}
			break;
		case Qt::EditRole:
			switch(index.column())
			{
				case COL_MEMORY:
					return (uint)entry.memory;
			}
			break;
		case Qt::TextAlignmentRole:
			switch(index.column())
			{
				case COL_NAME:
					return QVariant();
				case COL_NAMESPACE:
					return QVariant();
				case COL_RESTART_COUNT:
				case COL_LOAD:
				case COL_MEMORY:
					return int(Qt::AlignRight | Qt::AlignVCenter);
			}
			break;
		case Qt::BackgroundRole:
			switch(entry.state)
			{
				case rosmon_msgs::NodeState::RUNNING:
					return QVariant();
				case rosmon_msgs::NodeState::IDLE:
					return QBrush(QColor(200, 200, 200));
				case rosmon_msgs::NodeState::CRASHED:
					return QBrush(QColor(255, 100, 100));
				case rosmon_msgs::NodeState::WAITING:
					return QBrush(QColor(255, 255, 128));
			}
			break;
		case SortRole:
			switch(index.column())
			{
				case COL_NAME:
					return entry.name;
				case COL_NAMESPACE:
					return entry.ns;
				case COL_LOAD:
					return entry.load;
				case COL_MEMORY:
					return static_cast<unsigned int>(entry.memory);
				case COL_RESTART_COUNT:
					return entry.restartCount;
			}
		break;
	}

	return QVariant();
}

QVariant NodeModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if(role != Qt::DisplayRole || orientation != Qt::Horizontal)
		return QAbstractTableModel::headerData(section, orientation, role);

	switch(section)
	{
		case COL_NAME:          return "Node";
		case COL_NAMESPACE:     return "Namespace";
		case COL_LOAD:          return "CPU Load";
		case COL_MEMORY:        return "Memory";
		case COL_RESTART_COUNT: return "#Restarts";
	}

	return QVariant();
}

}

