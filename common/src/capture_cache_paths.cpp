#include "scan_tracking/common/capture_cache_paths.h"

#include <QCoreApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>

namespace scan_tracking::common {

QString defaultCaptureCacheRoot()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/ScanTracking_CaptureCache");
}

QString resolveCaptureCacheRoot(const QString& configuredRoot)
{
    const QString trimmed = configuredRoot.trimmed();
    if (trimmed.isEmpty()) {
        return defaultCaptureCacheRoot();
    }
    return QDir(trimmed).absolutePath();
}

QString ensureDirectoryExists(const QString& directoryPath)
{
    if (directoryPath.trimmed().isEmpty()) {
        return QString();
    }

    QDir dir;
    if (!dir.mkpath(directoryPath)) {
        return QString();
    }
    return QDir(directoryPath).absolutePath();
}

QString captureCacheMech3DDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_3d")));
}

QString captureCacheMech2DDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_2d")));
}

QString captureCacheHikMonoDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("hik_mono")));
}

QString captureCacheOrbbecDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("orbbec")));
}

QString captureCacheHikMonoCameraDir(const QString& root, const QString& cameraTag)
{
    const QString hikRoot = captureCacheHikMonoDir(root);
    if (hikRoot.isEmpty()) {
        return QString();
    }

    QString subDir;
    if (cameraTag.compare(QStringLiteral("hikA"), Qt::CaseInsensitive) == 0) {
        subDir = QStringLiteral("camera_a");
    } else if (cameraTag.compare(QStringLiteral("hikB"), Qt::CaseInsensitive) == 0) {
        subDir = QStringLiteral("camera_b");
    } else {
        subDir = cameraTag.trimmed();
    }

    if (subDir.isEmpty()) {
        return QString();
    }

    return ensureDirectoryExists(QDir(hikRoot).absoluteFilePath(subDir));
}

QString buildCaptureTimestamp()
{
    return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
}

QString buildRunCaptureRoot(quint32 taskId, const QString& timestamp)
{
    const QString ts = timestamp.trimmed().isEmpty() ? buildCaptureTimestamp() : timestamp;
    const QString outputRoot = QCoreApplication::applicationDirPath() + QStringLiteral("/output");
    const QString runDirName = QStringLiteral("run_%1_%2").arg(taskId).arg(ts);
    return ensureDirectoryExists(QDir(outputRoot).absoluteFilePath(runDirName));
}

}  // namespace scan_tracking::common
