// List of all rosmon instances on the server
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef ROSMON_NODE_MODEL_H
#define ROSMON_NODE_MODEL_H

#include <QAbstractListModel>
#include <QStringList>

namespace rosmon
{

class ROSMonModel : public QAbstractListModel
{
Q_OBJECT
public:
	ROSMonModel(QObject* parent = nullptr);
	~ROSMonModel() override = default;

	int rowCount(const QModelIndex & parent) const override;
	QVariant data(const QModelIndex & index, int role) const override;
private Q_SLOTS:
	void updateData();
private:
	QStringList m_data;
};

}

#endif
