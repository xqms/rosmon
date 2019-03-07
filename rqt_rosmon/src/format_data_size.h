// QLocale::formatDataSize backported from Qt 5.10
// Ported by Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef FORMAT_DATA_SIZE_H
#define FORMAT_DATA_SIZE_H

#include <QString>

namespace rqt_rosmon
{

QString formattedDataSize(qint64 bytes, int precision);

}

#endif
