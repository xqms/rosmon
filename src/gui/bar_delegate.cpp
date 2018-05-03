// Renders a colored bar graph behind the data
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "bar_delegate.h"

#include <QPainter>

namespace rosmon
{

BarDelegate::BarDelegate(QObject* parent)
 : QStyledItemDelegate(parent)
 , m_min(0.0)
 , m_max(100.0)
{
}

void BarDelegate::setRange(double min, double max)
{
	m_min = min;
	m_max = max;
}

void BarDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	double data;

	QVariant var = index.data(Qt::EditRole);
	if(var.isValid())
		data = var.toDouble();
	else
		data = index.data(Qt::DisplayRole).toDouble();

	double alpha = std::min(1.0, std::max(0.0, (data - m_min) / (m_max - m_min)));

	QRect rect = option.rect;
	rect.setWidth(alpha * rect.width());

	QColor color(
		alpha * 255,
		(1.0f - alpha) * 255,
		0
// 		(1.0f - alpha) * 255
	);

	painter->fillRect(rect, color);

	QStyledItemDelegate::paint(painter, option, index);
}

}
