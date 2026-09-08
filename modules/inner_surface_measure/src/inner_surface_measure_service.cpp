#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QSaveFile>
#include <QtCore/QTextStream>

#include <mutex>
#include <string>

#include "InnerSurfaceMeasure.h"

namespace scan_tracking::inner_surface_measure {
namespace {
enum ErrorCode { kInvalidArg = 1, kNotInitialized = 2, kConfig = 3, kCloud = 4, kMeasure = 6, kInternal = 7 };

void FillError(InnerSurfaceMeasureError* error, int status, const QString& message)
{
    if (error) { error->statusCode = status; error->message = message; }
}
void DestroyContext(InnerSurfaceMeasure*& ctx)
{
    if (ctx) { DestroyInnerSurfaceMeasure(ctx); ctx = nullptr; }
}
void FillFrameResult(const ism::MeasurementResult& src, InnerSurfaceFrameMeasurement* dst)
{
    if (!dst) return;
    dst->diameterMm = src.diameter;
    dst->circumferenceMm = src.circumference;
    dst->sectionRoundness[0] = src.sectionRoundness[0];
    dst->sectionRoundness[1] = src.sectionRoundness[1];
    dst->sectionRoundness[2] = src.sectionRoundness[2];
    dst->averageRoundness = src.averageRoundness;
    dst->icpFitnessScore = 0.0;
    dst->icpConverged = src.roundnessValid;
    dst->usedPointCount = src.usedPointCount;
    dst->valid = src.roundnessValid && src.diameter > 0.0;
}
bool WritePcd(const QString& path, const float* xyz, size_t count, QString* error)
{
    if (!xyz || count == 0) { if (error) *error = QStringLiteral("Point cloud is empty"); return false; }
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) { if (error) *error = QStringLiteral("Cannot write point cloud: %1").arg(path); return false; }
    QTextStream out(&file);
    out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
        << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count << "\nDATA ascii\n";
    for (size_t i = 0; i < count; ++i) out << xyz[i * 3] << ' ' << xyz[i * 3 + 1] << ' ' << xyz[i * 3 + 2] << '\n';
    if (!file.commit()) { if (error) *error = QStringLiteral("Cannot commit point cloud: %1").arg(path); return false; }
    return true;
}
QString ResolvePath(const QString& baseDir, const QString& value)
{
    const QFileInfo info(value);
    return info.isAbsolute() ? info.absoluteFilePath() : QFileInfo(baseDir, value).absoluteFilePath();
}
bool WriteRuntimeIni(const QString& sourcePath, const QString& outputPath, const QString& templatePath,
                     const QString& frame1Path, const QString& frame2Path, double lengthMm, QString* error)
{
    QFile source(sourcePath);
    if (!source.open(QIODevice::ReadOnly | QIODevice::Text)) { if (error) *error = QStringLiteral("Cannot read config: %1").arg(sourcePath); return false; }
    QString text = QString::fromUtf8(source.readAll());
    auto replaceKey = [&text](const QString& key, const QString& value) {
        QRegularExpression re(QStringLiteral("(?m)^(\\s*%1\\s*=).*?$").arg(QRegularExpression::escape(key)));
        text.replace(re, QStringLiteral("\\1 %2").arg(value));
    };
    replaceKey(QStringLiteral("templateCloud"), QDir::toNativeSeparators(templatePath));
    replaceKey(QStringLiteral("frame1"), QDir::toNativeSeparators(frame1Path));
    replaceKey(QStringLiteral("frame2"), QDir::toNativeSeparators(frame2Path));
    if (lengthMm > 0.0) replaceKey(QStringLiteral("containerLength"), QString::number(lengthMm, 'f', 6));
    QSaveFile output(outputPath);
    if (!output.open(QIODevice::WriteOnly | QIODevice::Text)) { if (error) *error = QStringLiteral("Cannot write runtime config: %1").arg(outputPath); return false; }
    output.write(text.toUtf8());
    return output.commit();
}
}  // namespace

struct InnerSurfaceMeasureService::Impl { mutable std::mutex mutex; InnerSurfaceMeasure* ctx = nullptr; QString configPath; };
InnerSurfaceMeasureService::InnerSurfaceMeasureService() : m_impl(new Impl) {}
InnerSurfaceMeasureService::~InnerSurfaceMeasureService() { shutdown(); delete m_impl; m_impl = nullptr; }
QString InnerSurfaceMeasureService::defaultConfigPath() { return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/inner_surface_measure/config.ini")); }
bool InnerSurfaceMeasureService::isReady() const { if (!m_impl) return false; std::lock_guard<std::mutex> lock(m_impl->mutex); return m_impl->ctx != nullptr; }
QString InnerSurfaceMeasureService::configPath() const { return m_impl ? m_impl->configPath : QString(); }

bool InnerSurfaceMeasureService::initializeFromIni(const QString& configPath, InnerSurfaceMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex); DestroyContext(m_impl->ctx); m_impl->configPath.clear();
    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) { FillError(error, kConfig, QStringLiteral("Config not found: %1").arg(resolved)); return false; }
    InnerSurfaceMeasure* ctx = CreateInnerSurfaceMeasure(); std::string detail;
    const bool ok = ctx && ctx->LoadIni(QDir::toNativeSeparators(resolved).toStdString(), &detail);
    if (!ok) { DestroyContext(ctx); FillError(error, kConfig, QString::fromStdString(detail)); return false; }
    m_impl->ctx = ctx; m_impl->configPath = resolved; return true;
}
bool InnerSurfaceMeasureService::initialize(const InnerSurfaceConfig&, const float* templateXyz, size_t templateCount, InnerSurfaceMeasureError* error)
{
    if (!templateXyz || templateCount == 0) { FillError(error, kInvalidArg, QStringLiteral("Template cloud is empty")); return false; }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    const QString baseConfig = defaultConfigPath();
    if (!QFileInfo::exists(baseConfig)) { FillError(error, kConfig, QStringLiteral("Config not found: %1").arg(baseConfig)); return false; }
    const QString root = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/inner_surface_measure/runtime"));
    QDir().mkpath(root);
    const QString templ = QDir(root).filePath(QStringLiteral("template.pcd"));
    const QString ini = QDir(root).filePath(QStringLiteral("template_runtime.ini"));
    QString detail;
    if (!WritePcd(templ, templateXyz, templateCount, &detail) ||
        !WriteRuntimeIni(baseConfig, ini, templ, templ, templ, 0.0, &detail)) {
        FillError(error, kConfig, detail);
        return false;
    }
    InnerSurfaceMeasure* ctx = CreateInnerSurfaceMeasure();
    std::string loadError;
    if (!ctx || !ctx->LoadIni(QDir::toNativeSeparators(ini).toStdString(), &loadError)) {
        DestroyContext(ctx);
        FillError(error, kConfig, QString::fromStdString(loadError));
        return false;
    }
    m_impl->ctx = ctx;
    m_impl->configPath = ini;
    return true;
}
void InnerSurfaceMeasureService::shutdown() { if (!m_impl) return; std::lock_guard<std::mutex> lock(m_impl->mutex); DestroyContext(m_impl->ctx); m_impl->configPath.clear(); }
bool InnerSurfaceMeasureService::measureFrame(const float*, size_t, InnerSurfaceFrameMeasurement*, InnerSurfaceMeasureError* error)
{ FillError(error, kInternal, QStringLiteral("The path4 V2 algorithm requires two frames")); return false; }
bool InnerSurfaceMeasureService::measureTwoFramesAverage(const float* a, size_t ac, const float* b, size_t bc,
    InnerSurfaceAverageMeasurement* avg, InnerSurfaceFrameMeasurement* f1, InnerSurfaceFrameMeasurement* f2, InnerSurfaceMeasureError* error)
{ return measureTwoFramesAverageWithLength(a, ac, b, bc, 0.0, avg, f1, f2, error); }

bool InnerSurfaceMeasureService::measureTwoFramesAverageWithLength(const float* frame1Xyz, size_t frame1Count,
    const float* frame2Xyz, size_t frame2Count, double measuredLengthMm, InnerSurfaceAverageMeasurement* outAverage,
    InnerSurfaceFrameMeasurement* outFrame1, InnerSurfaceFrameMeasurement* outFrame2, InnerSurfaceMeasureError* error)
{
    if (!outAverage || !frame1Xyz || !frame2Xyz || frame1Count == 0 || frame2Count == 0) { FillError(error, kInvalidArg, QStringLiteral("Invalid path4 frame or output")); return false; }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (!m_impl->ctx || m_impl->configPath.isEmpty()) { FillError(error, kNotInitialized, QStringLiteral("InnerSurfaceMeasureService not initialized")); return false; }
    const QString root = QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/inner_surface_measure/runtime")); QDir().mkpath(root);
    const QString p1 = QDir(root).filePath(QStringLiteral("frame1.pcd")); const QString p2 = QDir(root).filePath(QStringLiteral("frame2.pcd")); const QString ini = QDir(root).filePath(QStringLiteral("runtime.ini")); QString detail;
    if (!WritePcd(p1, frame1Xyz, frame1Count, &detail) || !WritePcd(p2, frame2Xyz, frame2Count, &detail)) { FillError(error, kCloud, detail); return false; }
    QFile source(m_impl->configPath); if (!source.open(QIODevice::ReadOnly | QIODevice::Text)) { FillError(error, kConfig, QStringLiteral("Cannot read config")); return false; }
    const QString configText = QString::fromUtf8(source.readAll()); const auto match = QRegularExpression(QStringLiteral("(?m)^\\s*templateCloud\\s*=\\s*(.*?)\\s*$")).match(configText);
    if (!match.hasMatch()) { FillError(error, kConfig, QStringLiteral("templateCloud is missing from config")); return false; }
    const QString templ = ResolvePath(QFileInfo(m_impl->configPath).absolutePath(), match.captured(1).trimmed());
    if (!WriteRuntimeIni(m_impl->configPath, ini, templ, p1, p2, measuredLengthMm, &detail)) { FillError(error, kConfig, detail); return false; }
    InnerSurfaceMeasure* run = CreateInnerSurfaceMeasure(); std::string runError; const bool loaded = run && run->LoadIni(QDir::toNativeSeparators(ini).toStdString(), &runError);
    ism::MeasurementResult frames[2]{}; double diameter = 0.0, circumference = 0.0, roundness = 0.0, volume = 0.0;
    const bool measured = loaded && run->Measure(frames, &diameter, &circumference, &roundness, &volume, &runError); if (run) DestroyInnerSurfaceMeasure(run);
    if (!measured) { FillError(error, kMeasure, QString::fromStdString(runError)); return false; }
    outAverage->diameterMm = diameter; outAverage->circumferenceMm = circumference; outAverage->roundness = roundness; outAverage->volumeLiters = volume;
    outAverage->containerLengthMm = measuredLengthMm > 0.0 ? measuredLengthMm : m_impl->ctx->ContainerLength(); outAverage->valid = diameter > 0.0 && frames[0].roundnessValid && frames[1].roundnessValid;
    FillFrameResult(frames[0], outFrame1); FillFrameResult(frames[1], outFrame2); return outAverage->valid;
}
}  // namespace scan_tracking::inner_surface_measure
