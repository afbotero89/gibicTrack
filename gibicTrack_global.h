#ifndef GIBICTRACK_GLOBAL_H
#define GIBICTRACK_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(GIBICTRACK_LIBRARY)
#  define GIBICTRACKSHARED_EXPORT Q_DECL_EXPORT
#else
#  define GIBICTRACKSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // GIBICTRACK_GLOBAL_H
