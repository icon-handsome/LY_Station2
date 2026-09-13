#include "scan_tracking/flow_control/inspection_types.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QHash>
#include <QtCore/QTextStream>

namespace scan_tracking {
namespace flow_control {

namespace {

constexpr const char* kTwelveMetricHeaderPrefix =
    "测试目标指标（12项，左右侧咬边合并计项）:";

void appendKeyValue(QString* out, const QString& key, const QString& value)
{
    *out += key;
    *out += QLatin1Char('=');
    *out += value;
    *out += QLatin1Char('\n');
}

void appendKeyValue(QString* out, const QString& key, qint64 value)
{
    appendKeyValue(out, key, QString::number(value));
}

void appendKeyValue(QString* out, const QString& key, double value)
{
    appendKeyValue(out, key, QString::number(value, 'f', 6));
}

QString formatTwelveMetricHeader(const InspectionMeasurement& m)
{
    QString text;
    text.reserve(512);
    text += QString::fromUtf8(kTwelveMetricHeaderPrefix);
    text += QLatin1Char('\n');
    text += QStringLiteral("1. 焊缝错边量 = %1 mm\n").arg(m.mismatchMm);
    text += QStringLiteral("2. 焊缝余高 = %1 mm\n").arg(m.reinforcementMm);
    text += QStringLiteral("3. 母材拟合线夹角 = %1 deg\n").arg(m.includedAngleDeg);
    text += QStringLiteral("4. 棱角度 = %1 mm\n").arg(m.angularityMm);
    text += QStringLiteral("5. 左右咬边深度 = 左 %1 mm，右 %2 mm\n")
                .arg(m.leftUndercutMm)
                .arg(m.rightUndercutMm);
    text += QStringLiteral("6. 左右咬边长度 = 左 %1 mm，右 %2 mm\n")
                .arg(m.leftUndercutLengthMm)
                .arg(m.rightUndercutLengthMm);
    text += QStringLiteral("7. 筒体焊缝附近平均厚度 = %1 mm\n").arg(m.thicknessMm);
    text += QStringLiteral("8. 内径 = %1 mm\n").arg(m.innerDiameterMm);
    text += QStringLiteral("9. 内周长 = %1 mm\n").arg(m.innerCircumferenceMm);
    text += QStringLiteral("10. 内表面圆度 = %1 mm\n").arg(m.innerRoundness);
    text += QStringLiteral("11. 筒体总长 = %1 mm\n").arg(m.lengthMm);
    text += QStringLiteral("12. 容积 = %1 L\n\n").arg(m.volumeLiters);
    return text;
}

/// 去掉文件顶部 12 项汇总头，只保留 path 结果块正文。
QString stripTwelveMetricHeader(const QString& content)
{
    const QString trimmed = content;
    const int marker = trimmed.indexOf(QString::fromUtf8(kTwelveMetricHeaderPrefix));
    if (marker < 0) {
        return trimmed;
    }

    const int blockStart = trimmed.indexOf(QStringLiteral("========"), marker);
    if (blockStart >= 0) {
        return trimmed.mid(blockStart);
    }

    // 仅有汇总头、尚无 path 块时视为空正文。
    return QString();
}

QString algorithmFromBannerLine(const QString& line)
{
    const QString key = QStringLiteral("algorithm=");
    const int begin = line.indexOf(key);
    if (begin < 0) {
        return QString();
    }
    const int valueBegin = begin + key.size();
    int valueEnd = line.indexOf(QLatin1Char(' '), valueBegin);
    if (valueEnd < 0) {
        valueEnd = line.size();
    }
    return line.mid(valueBegin, valueEnd - valueBegin).trimmed();
}

double kvDouble(const QHash<QString, QString>& kv, const QString& key)
{
    bool ok = false;
    const double value = kv.value(key).toDouble(&ok);
    return ok ? value : 0.0;
}

int kvInt(const QHash<QString, QString>& kv, const QString& key)
{
    bool ok = false;
    const int value = kv.value(key).toInt(&ok);
    return ok ? value : 0;
}

void mergeSuccessfulPathBlock(
    InspectionMeasurement* aggregated,
    const QString& algorithm,
    const QHash<QString, QString>& kv)
{
    if (aggregated == nullptr) {
        return;
    }

    if (algorithm == QLatin1String("weld_section")) {
        aggregated->mismatchMm = kvDouble(kv, QStringLiteral("mismatchMm"));
        aggregated->reinforcementMm = kvDouble(kv, QStringLiteral("reinforcementMm"));
        aggregated->angularityMm = kvDouble(kv, QStringLiteral("angularityMm"));
        aggregated->includedAngleDeg = kvDouble(kv, QStringLiteral("includedAngleDeg"));
        aggregated->leftUndercutMm = kvDouble(kv, QStringLiteral("leftUndercutMm"));
        aggregated->rightUndercutMm = kvDouble(kv, QStringLiteral("rightUndercutMm"));
        aggregated->maxUndercutMm = kvDouble(kv, QStringLiteral("maxUndercutMm"));
        aggregated->leftUndercutLengthMm =
            kvDouble(kv, QStringLiteral("leftUndercutLengthMm"));
        aggregated->rightUndercutLengthMm =
            kvDouble(kv, QStringLiteral("rightUndercutLengthMm"));
        aggregated->measuredSegmentCount =
            kvInt(kv, QStringLiteral("measuredSegmentCount"));
        return;
    }

    if (algorithm == QLatin1String("length_volume")) {
        const double lengthMm = kvDouble(kv, QStringLiteral("lengthMm"));
        if (lengthMm != 0.0) {
            aggregated->lengthMm = lengthMm;
        }
        return;
    }

    if (algorithm == QLatin1String("thickness_inner_surface")) {
        aggregated->thicknessMm = kvDouble(kv, QStringLiteral("thicknessMm"));
        aggregated->thicknessPairCount =
            kvInt(kv, QStringLiteral("thicknessPairCount"));
        aggregated->thicknessSuccessCount =
            kvInt(kv, QStringLiteral("thicknessSuccessCount"));
        aggregated->innerDiameterMm = kvDouble(kv, QStringLiteral("innerDiameterMm"));
        aggregated->innerCircumferenceMm =
            kvDouble(kv, QStringLiteral("innerCircumferenceMm"));
        aggregated->innerRoundness = kvDouble(kv, QStringLiteral("innerRoundness"));
        aggregated->volumeLiters = kvDouble(kv, QStringLiteral("volumeLiters"));
        aggregated->volumeRadiusMm = kvDouble(kv, QStringLiteral("volumeRadiusMm"));
        aggregated->innerSurfacePairCount =
            kvInt(kv, QStringLiteral("innerSurfacePairCount"));
        aggregated->innerSurfaceSuccessCount =
            kvInt(kv, QStringLiteral("innerSurfaceSuccessCount"));
        const double lengthMm = kvDouble(kv, QStringLiteral("lengthMm"));
        if (lengthMm != 0.0) {
            aggregated->lengthMm = lengthMm;
        }
        return;
    }

    if (algorithm == QLatin1String("code_read")) {
        const QString codeValue = kv.value(QStringLiteral("codeValue")).trimmed();
        if (!codeValue.isEmpty()) {
            aggregated->codeValue = codeValue;
        }
    }
}

InspectionMeasurement aggregateMeasurementFromResultBody(const QString& body)
{
    InspectionMeasurement aggregated;
    const QStringList lines = body.split(QLatin1Char('\n'));
    QString currentAlgorithm;
    QHash<QString, QString> currentKv;
    bool inBlock = false;

    auto flushBlock = [&]() {
        if (!inBlock) {
            return;
        }
        if (kvInt(currentKv, QStringLiteral("resultCode")) == 1) {
            mergeSuccessfulPathBlock(&aggregated, currentAlgorithm, currentKv);
        }
        currentAlgorithm.clear();
        currentKv.clear();
        inBlock = false;
    };

    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();
        if (line.startsWith(QStringLiteral("========")) &&
            line.contains(QStringLiteral("pathId="))) {
            flushBlock();
            currentAlgorithm = algorithmFromBannerLine(line);
            inBlock = true;
            continue;
        }
        if (!inBlock || line.isEmpty()) {
            continue;
        }
        const int eq = line.indexOf(QLatin1Char('='));
        if (eq <= 0) {
            continue;
        }
        currentKv.insert(line.left(eq).trimmed(), line.mid(eq + 1).trimmed());
    }
    flushBlock();
    return aggregated;
}

bool readEntireTextFile(const QString& filePath, QString* content, QString* errorMessage)
{
    QFile file(filePath);
    if (!file.exists() || file.size() == 0) {
        if (content != nullptr) {
            content->clear();
        }
        return true;
    }
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法读取 %1：%2")
                                .arg(filePath, file.errorString());
        }
        return false;
    }
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    if (content != nullptr) {
        *content = stream.readAll();
    }
    return true;
}

bool writeEntireTextFile(const QString& filePath, const QString& content, QString* errorMessage)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法写入 %1：%2")
                                .arg(filePath, file.errorString());
        }
        return false;
    }
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << content;
    stream.flush();
    if (stream.status() != QTextStream::Ok) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("写入 %1 失败。").arg(filePath);
        }
        return false;
    }
    return true;
}

}  // namespace

void appendInspectionMeasurementFields(
    QJsonObject& payload,
    const InspectionMeasurement& measurement,
    const QString& algorithm)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("qualityCode")] = measurement.qualityCode;
    const QString normalizedAlgorithm = algorithm.trimmed().toLower();

    if (normalizedAlgorithm == QLatin1String("weld_section")) {
        headMetrics[QStringLiteral("mismatchMm")] = measurement.mismatchMm;
        headMetrics[QStringLiteral("reinforcementMm")] = measurement.reinforcementMm;
        headMetrics[QStringLiteral("angularityMm")] = measurement.angularityMm;
        headMetrics[QStringLiteral("includedAngleDeg")] = measurement.includedAngleDeg;
        headMetrics[QStringLiteral("leftUndercutMm")] = measurement.leftUndercutMm;
        headMetrics[QStringLiteral("rightUndercutMm")] = measurement.rightUndercutMm;
        headMetrics[QStringLiteral("maxUndercutMm")] = measurement.maxUndercutMm;
        headMetrics[QStringLiteral("leftUndercutLengthMm")] = measurement.leftUndercutLengthMm;
        headMetrics[QStringLiteral("rightUndercutLengthMm")] = measurement.rightUndercutLengthMm;
        headMetrics[QStringLiteral("measuredSegmentCount")] = measurement.measuredSegmentCount;
    } else if (normalizedAlgorithm == QLatin1String("thickness_inner_surface")) {
        headMetrics[QStringLiteral("thicknessMm")] = measurement.thicknessMm;
        headMetrics[QStringLiteral("thickness_mm")] = measurement.thicknessMm;
        headMetrics[QStringLiteral("thicknessPairCount")] = measurement.thicknessPairCount;
        headMetrics[QStringLiteral("thicknessSuccessCount")] = measurement.thicknessSuccessCount;
        headMetrics[QStringLiteral("innerDiameterMm")] = measurement.innerDiameterMm;
        headMetrics[QStringLiteral("inner_diameter_mm")] = measurement.innerDiameterMm;
        headMetrics[QStringLiteral("innerCircumferenceMm")] = measurement.innerCircumferenceMm;
        headMetrics[QStringLiteral("inner_circumference_mm")] = measurement.innerCircumferenceMm;
        headMetrics[QStringLiteral("innerRoundness")] = measurement.innerRoundness;
        headMetrics[QStringLiteral("roundness_tol")] = measurement.innerRoundness;
        headMetrics[QStringLiteral("innerSurfacePairCount")] = measurement.innerSurfacePairCount;
        headMetrics[QStringLiteral("innerSurfaceSuccessCount")] = measurement.innerSurfaceSuccessCount;
        headMetrics[QStringLiteral("lengthMm")] = measurement.lengthMm;
        headMetrics[QStringLiteral("length_mm")] = measurement.lengthMm;
        headMetrics[QStringLiteral("volumeLiters")] = measurement.volumeLiters;
        headMetrics[QStringLiteral("volume_liters")] = measurement.volumeLiters;
        headMetrics[QStringLiteral("volumeRadiusMm")] = measurement.volumeRadiusMm;
        headMetrics[QStringLiteral("volume_radius_mm")] = measurement.volumeRadiusMm;
    } else if (normalizedAlgorithm == QLatin1String("length_volume")) {
        headMetrics[QStringLiteral("lengthMm")] = measurement.lengthMm;
        headMetrics[QStringLiteral("length_mm")] = measurement.lengthMm;
    } else if (normalizedAlgorithm == QLatin1String("code_read")) {
        if (!measurement.codeValue.isEmpty()) {
            headMetrics[QStringLiteral("codeValue")] = measurement.codeValue;
        }
    }
    payload[QStringLiteral("headMetrics")] = headMetrics;
}

QString formatInspectionResultTextBlock(const InspectionResult& result)
{
    QString text;
    text.reserve(512);
    text += QStringLiteral("======== pathId=%1 pathName=%2 algorithm=%3 ========\n")
                .arg(result.pathId)
                .arg(result.pathName.isEmpty() ? QStringLiteral("-") : result.pathName)
                .arg(result.algorithm.isEmpty() ? QStringLiteral("-") : result.algorithm);
    appendKeyValue(&text, QStringLiteral("time"),
                   QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")));
    appendKeyValue(&text, QStringLiteral("elapsedSec"),
                   QString::number(result.elapsedSeconds, 'f', 2));
    appendKeyValue(&text, QStringLiteral("resultCode"), static_cast<qint64>(result.resultCode));

    const bool success = result.resultCode == 1;
    if (!success) {
        // 失败：只记失败码与描述，不写测量数值结果。
        appendKeyValue(&text, QStringLiteral("ngReasonWord0"), static_cast<qint64>(result.ngReasonWord0));
        appendKeyValue(&text, QStringLiteral("ngReasonWord1"), static_cast<qint64>(result.ngReasonWord1));
        appendKeyValue(&text, QStringLiteral("message"),
                       result.message.isEmpty() ? QStringLiteral("-") : result.message);
        text += QLatin1Char('\n');
        return text;
    }

    appendKeyValue(&text, QStringLiteral("ngReasonWord0"), static_cast<qint64>(result.ngReasonWord0));
    appendKeyValue(&text, QStringLiteral("ngReasonWord1"), static_cast<qint64>(result.ngReasonWord1));
    appendKeyValue(&text, QStringLiteral("measureItemCount"), static_cast<qint64>(result.measureItemCount));
    appendKeyValue(&text, QStringLiteral("sourcePointCount"), static_cast<qint64>(result.sourcePointCount));
    appendKeyValue(&text, QStringLiteral("qualityCode"),
                   static_cast<qint64>(result.measurement.qualityCode));

    const QString algorithm = result.algorithm.trimmed();
    const auto& m = result.measurement;
    if (algorithm == QLatin1String("weld_section")) {
        appendKeyValue(&text, QStringLiteral("mismatchMm"), m.mismatchMm);
        appendKeyValue(&text, QStringLiteral("reinforcementMm"), m.reinforcementMm);
        appendKeyValue(&text, QStringLiteral("angularityMm"), m.angularityMm);
        appendKeyValue(&text, QStringLiteral("includedAngleDeg"), m.includedAngleDeg);
        appendKeyValue(&text, QStringLiteral("leftUndercutMm"), m.leftUndercutMm);
        appendKeyValue(&text, QStringLiteral("rightUndercutMm"), m.rightUndercutMm);
        appendKeyValue(&text, QStringLiteral("maxUndercutMm"), m.maxUndercutMm);
        appendKeyValue(&text, QStringLiteral("leftUndercutLengthMm"), m.leftUndercutLengthMm);
        appendKeyValue(&text, QStringLiteral("rightUndercutLengthMm"), m.rightUndercutLengthMm);
        appendKeyValue(&text, QStringLiteral("measuredSegmentCount"),
                       static_cast<qint64>(m.measuredSegmentCount));
    } else if (algorithm == QLatin1String("thickness_inner_surface")) {
        appendKeyValue(&text, QStringLiteral("thicknessMm"), m.thicknessMm);
        appendKeyValue(&text, QStringLiteral("thicknessPairCount"),
                       static_cast<qint64>(m.thicknessPairCount));
        appendKeyValue(&text, QStringLiteral("thicknessSuccessCount"),
                       static_cast<qint64>(m.thicknessSuccessCount));
        appendKeyValue(&text, QStringLiteral("innerDiameterMm"), m.innerDiameterMm);
        appendKeyValue(&text, QStringLiteral("innerCircumferenceMm"), m.innerCircumferenceMm);
        appendKeyValue(&text, QStringLiteral("innerRoundness"), m.innerRoundness);
        appendKeyValue(&text, QStringLiteral("lengthMm"), m.lengthMm);
        appendKeyValue(&text, QStringLiteral("volumeLiters"), m.volumeLiters);
        appendKeyValue(&text, QStringLiteral("volumeRadiusMm"), m.volumeRadiusMm);
        appendKeyValue(&text, QStringLiteral("innerSurfacePairCount"),
                       static_cast<qint64>(m.innerSurfacePairCount));
        appendKeyValue(&text, QStringLiteral("innerSurfaceSuccessCount"),
                       static_cast<qint64>(m.innerSurfaceSuccessCount));
    } else if (algorithm == QLatin1String("length_volume")) {
        // path3 只负责筒体长度检测；其它拟合字段不写入结果文本。
        appendKeyValue(&text, QStringLiteral("lengthMm"), m.lengthMm);
    } else if (algorithm == QLatin1String("code_read") && !m.codeValue.isEmpty()) {
        appendKeyValue(&text, QStringLiteral("codeValue"), m.codeValue);
    } else {
        // 未知/通用：尽量写出非零测量字段，避免丢信息。
        if (m.mismatchMm != 0.0) {
            appendKeyValue(&text, QStringLiteral("mismatchMm"), m.mismatchMm);
        }
        if (m.thicknessMm != 0.0) {
            appendKeyValue(&text, QStringLiteral("thicknessMm"), m.thicknessMm);
        }
        if (m.lengthMm != 0.0) {
            appendKeyValue(&text, QStringLiteral("lengthMm"), m.lengthMm);
        }
        if (m.volumeLiters != 0.0) {
            appendKeyValue(&text, QStringLiteral("volumeLiters"), m.volumeLiters);
        }
        if (!m.codeValue.isEmpty()) {
            appendKeyValue(&text, QStringLiteral("codeValue"), m.codeValue);
        }
    }

    appendKeyValue(&text, QStringLiteral("message"),
                   result.message.isEmpty() ? QStringLiteral("-") : result.message);
    text += QLatin1Char('\n');
    return text;
}

bool appendInspectionResultToRunFile(
    const QString& runCaptureRoot,
    const InspectionResult& result,
    QString* errorMessage)
{
    const QString root = runCaptureRoot.trimmed();
    if (root.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("runCaptureRoot 为空，无法写 result.txt。");
        }
        return false;
    }

    if (!QDir(root).exists() && !QDir().mkpath(root)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法创建 run 目录：%1").arg(root);
        }
        return false;
    }

    const QString filePath = QDir(root).absoluteFilePath(QStringLiteral("result.txt"));
    // 与 path_{id}/ 并列：output/run_*/result.txt
    // 每次追加后按全部成功 path 块重写 12 项汇总头，避免首条焊缝结果把 7–12 永久写成 0。
    QString existing;
    if (!readEntireTextFile(filePath, &existing, errorMessage)) {
        return false;
    }

    QString body = stripTwelveMetricHeader(existing);
    if (!body.isEmpty() && !body.endsWith(QLatin1Char('\n'))) {
        body += QLatin1Char('\n');
    }
    body += formatInspectionResultTextBlock(result);

    const InspectionMeasurement aggregated = aggregateMeasurementFromResultBody(body);
    const QString rewritten = formatTwelveMetricHeader(aggregated) + body;
    return writeEntireTextFile(filePath, rewritten, errorMessage);
}

}  // namespace flow_control
}  // namespace scan_tracking
