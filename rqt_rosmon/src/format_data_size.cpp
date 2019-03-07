// QLocale::formatDataSize backported from Qt 5.10
// Ported by Max Schwarz <max.schwarz@uni-bonn.de>

#include "format_data_size.h"

#include <cmath>

#include <QtAlgorithms>
#include <QLocale>

namespace rqt_rosmon
{

static inline uint countLeadingZeroBits(quint64 v)
{
	return __builtin_clzll(v);
}

QString formattedDataSize(qint64 bytes, int precision)
{
	int power, base = 1000;
	if (!bytes) {
		power = 0;
	} else { // Compute log2(bytes) / 10:
		power = int((63 - countLeadingZeroBits(quint64(qAbs(bytes)))) / 10);
		base = 1024;
	}

	QLocale locale = QLocale::system();

	// Only go to doubles if we'll be using a quantifier:
	const QString number = power
		? locale.toString(bytes / std::pow(double(base), power), 'f', qMin(precision, 3 * power))
		: locale.toString(bytes);

	// We don't support sizes in units larger than exbibytes because
	// the number of bytes would not fit into qint64.
	Q_ASSERT(power <= 6 && power >= 0);
	QString unit;
	if (power > 0) {
		switch(power)
		{
			case 1: unit = "KiB"; break;
			case 2: unit = "MiB"; break;
			case 3: unit = "GiB"; break;
			case 4: unit = "TiB"; break;
			case 5: unit = "PiB"; break;
			case 6: unit = "EiB"; break;
		}
	} else {
		unit = "B";
	}

	return number + QLatin1Char(' ') + unit;
}

}
