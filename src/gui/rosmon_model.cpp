// List of all rosmon instances on the server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "rosmon_model.h"

#include <QTimer>

#include <ros/master.h>
#include <ros/names.h>

namespace rosmon
{

ROSMonModel::ROSMonModel(QObject* parent)
 : QAbstractListModel(parent)
{
	m_data << "[auto]";

	auto updateTimer = new QTimer(this);
	connect(updateTimer, SIGNAL(timeout()), SLOT(updateData()));
	updateTimer->start(2000);
}

int ROSMonModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_data.count();
}

QVariant ROSMonModel::data(const QModelIndex& index, int role) const
{
	int row = index.row();

	switch(role)
	{
		case Qt::DisplayRole:
		case Qt::EditRole:
			return m_data[row];
	}

	return QVariant();
}

void ROSMonModel::updateData()
{
	std::vector<ros::master::TopicInfo> topics;
	if(!ros::master::getTopics(topics))
		return;

	QStringList nodeList;

	for(auto& topic : topics)
	{
		if(topic.datatype != "rosmon/State")
			continue;

		nodeList << QString::fromStdString(ros::names::parentNamespace(topic.name));
	}

	qSort(nodeList);

	nodeList.prepend("[auto]");

	int oldIdx = 1;
	int newIdx = 1;

	while(oldIdx < m_data.size() || newIdx < nodeList.size())
	{
		if(oldIdx == m_data.size())
		{
			beginInsertRows(QModelIndex(), oldIdx, oldIdx);
			m_data.append(nodeList[newIdx]);
			endInsertRows();
			oldIdx += 1;
			newIdx += 1;
			continue;
		}

		if(newIdx == nodeList.size())
		{
			beginRemoveRows(QModelIndex(), oldIdx, oldIdx);
			m_data.erase(m_data.begin() + oldIdx);
			endRemoveRows();
			continue;
		}

		auto oldT = m_data[oldIdx];
		auto newT = nodeList[newIdx];

		if(oldT == newT)
		{
			oldIdx++;
			newIdx++;
		}
		else if(oldT < newT)
		{
			beginRemoveRows(QModelIndex(), oldIdx, oldIdx);
			m_data.erase(m_data.begin() + oldIdx);
			endRemoveRows();
		}
		else
		{
			beginInsertRows(QModelIndex(), oldIdx, oldIdx);
			m_data.insert(oldIdx, newT);
			endInsertRows();
			oldIdx++;
			newIdx++;
		}
	}
}

}
