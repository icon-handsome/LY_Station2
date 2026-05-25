#include "scan_tracking/vision/hik_mono_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLoggingCategory>

Q_LOGGING_CATEGORY(LOG_HIK_MONO_IO, "vision.hik_mono_io")

namespace scan_tracking::vision {

namespace {

constexpr int kPgmMaxGray = 255;

}  // namespace

QString buildSegmentHikMonoPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& cameraTag,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheHikMonoDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_HIK_MONO_IO).noquote() << "无法创建 hik_mono 缓存目录";
        return QString();
    }

    const QString ts =
        timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
    const QString fileName = QStringLiteral("segment_%1_task%2_%3_%4.pgm")
                                 .arg(segmentIndex)
                                 .arg(taskId)
                                 .arg(ts)
                                 .arg(cameraTag);
    return QDir(baseDir).absoluteFilePath(fileName);
}

bool saveHikMonoFrameToPgm(const HikMonoFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        qWarning(LOG_HIK_MONO_IO) << "saveHikMonoFrameToPgm: invalid frame or path";
        return false;
    }

    if (frame.stride <= 0) {
        qWarning(LOG_HIK_MONO_IO) << "saveHikMonoFrameToPgm: invalid stride";
        return false;
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    QFile file(absolutePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning(LOG_HIK_MONO_IO).noquote()
            << "saveHikMonoFrameToPgm: cannot open" << absolutePath;
        return false;
    }

    const QByteArray header = QStringLiteral("P5\n%1 %2\n%3\n")
                                  .arg(frame.width)
                                  .arg(frame.height)
                                  .arg(kPgmMaxGray)
                                  .toLatin1();
    if (file.write(header) != header.size()) {
        qWarning(LOG_HIK_MONO_IO) << "saveHikMonoFrameToPgm: failed to write header";
        return false;
    }

    const auto& pixels = *frame.pixels;
    const int rowBytes = frame.width;
    const int stride = frame.stride > 0 ? frame.stride : frame.width;

    for (int row = 0; row < frame.height; ++row) {
        const int offset = row * stride;
        if (offset + rowBytes > static_cast<int>(pixels.size())) {
            qWarning(LOG_HIK_MONO_IO) << "saveHikMonoFrameToPgm: pixel buffer underrun";
            return false;
        }
        if (file.write(reinterpret_cast<const char*>(pixels.data() + offset), rowBytes) != rowBytes) {
            qWarning(LOG_HIK_MONO_IO) << "saveHikMonoFrameToPgm: failed to write row";
            return false;
        }
    }

    file.close();

    qInfo(LOG_HIK_MONO_IO).noquote()
        << "PGM saved:" << absolutePath
        << "size=" << frame.width << "x" << frame.height;

    return true;
}

void releaseHikMonoFrameBuffers(HikMonoFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pixels.reset();
}

}  // namespace scan_tracking::vision
