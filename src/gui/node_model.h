// Qt model for nodes controlled by a rosmon instance
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef NODE_MODEL_H
#define NODE_MODEL_H

#include <QAbstractTableModel>

#include <rosmon/State.h>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

namespace rosmon
{

class NodeModel : public QAbstractTableModel
{
Q_OBJECT
public:
	enum Column
	{
		COL_NAME,
		COL_RESTART_COUNT,
		COL_LOAD,
		COL_MEMORY,

		COL_COUNT
	};

	explicit NodeModel(ros::NodeHandle& nh, QObject* parent = nullptr);
	~NodeModel() override = default;

	int rowCount(const QModelIndex & parent) const override;
	int columnCount(const QModelIndex & parent) const override;
	QVariant data(const QModelIndex & index, int role) const override;

	QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

	inline QString namespaceString() const
	{ return m_namespace; }

	QString nodeName(int row) const;
public Q_SLOTS:
	void setNamespace(const QString& ns);
	void unsubscribe();
Q_SIGNALS:
	void stateReceived(const rosmon::StateConstPtr& state);
private Q_SLOTS:
	void updateState(const rosmon::StateConstPtr& state);
private:
	struct Entry
	{
		inline bool operator<(const Entry& other) const
		{ return name < other.name; }

		QString name;
		int state;
		int restartCount;
		double load;
		uint64_t memory;
	};

	ros::NodeHandle m_nh;

	QString m_namespace;

	std::vector<Entry> m_entries;

	ros::Subscriber m_sub_state;
};

}

#endif
