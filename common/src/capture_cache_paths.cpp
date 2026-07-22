#include "scan_tracking/common/capture_cache_paths.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QLoggingCategory>

Q_LOGGING_CATEGORY(LOG_CAPTURE_PATHS, "common.capture_paths")

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
        qWarning(LOG_CAPTURE_PATHS).noquote()
            << QStringLiteral("创建目录失败：") << directoryPath;
        return QString();
    }
    return QDir(directoryPath).absolutePath();
}

QString capturePointDirectory(
    const QString& runRoot,
    int pathId,
    const QString& deviceTag,
    int pointIndex)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(runRoot));
    if (resolved.isEmpty()) {
        return QString();
    }

    const int safePathId = pathId > 0 ? pathId : 0;
    const int safePointIndex = pointIndex > 0 ? pointIndex : 0;
    QString tag = deviceTag.trimmed().toLower();
    if (tag.isEmpty()) {
        tag = QStringLiteral("arm");
    }

    const QString pathDir = ensureDirectoryExists(
        QDir(resolved).absoluteFilePath(QStringLiteral("path_%1").arg(safePathId)));
    if (pathDir.isEmpty()) {
        return QString();
    }
    const QString deviceDir = ensureDirectoryExists(QDir(pathDir).absoluteFilePath(tag));
    if (deviceDir.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(
        QDir(deviceDir).absoluteFilePath(QString::number(safePointIndex)));
}

QString captureCacheMech3DDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_3d")));
}

QString captureCacheMechTextureDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_texture")));
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
    QString appDir = QCoreApplication::applicationDirPath().trimmed();
    if (appDir.isEmpty()) {
        appDir = QDir::currentPath();
        qWarning(LOG_CAPTURE_PATHS).noquote()
            << QStringLiteral("applicationDirPath 为空，回退到当前工作目录：") << appDir;
    }
    const QString outputRoot = QDir(appDir).absoluteFilePath(QStringLiteral("output"));
    const QString runDirName = QStringLiteral("run_%1_%2").arg(taskId).arg(ts);
    const QString runPath = QDir(outputRoot).absoluteFilePath(runDirName);
    const QString created = ensureDirectoryExists(runPath);
    if (created.isEmpty()) {
        qWarning(LOG_CAPTURE_PATHS).noquote()
            << QStringLiteral("buildRunCaptureRoot 失败 taskId=") << taskId
            << QStringLiteral(" path=") << runPath;
    }
    return created;
}

}  // namespace scan_tracking::common
