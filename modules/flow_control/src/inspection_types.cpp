#include "scan_tracking/flow_control/inspection_types.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QTextStream>

namespace scan_tracking {
namespace flow_control {

namespace {

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

}  // namespace

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("qualityCode")] = measurement.qualityCode;
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
    headMetrics[QStringLiteral("fittedOuterRadiusMm")] = measurement.fittedOuterRadiusMm;
    headMetrics[QStringLiteral("fitted_outer_radius_mm")] = measurement.fittedOuterRadiusMm;
    headMetrics[QStringLiteral("containerLeftEndPositionMm")] =
        measurement.containerLeftEndPositionMm;
    headMetrics[QStringLiteral("containerRightEndPositionMm")] =
        measurement.containerRightEndPositionMm;
    headMetrics[QStringLiteral("containerIcpFitness")] = measurement.containerIcpFitness;
    headMetrics[QStringLiteral("containerFittedRadiusMm")] = measurement.containerFittedRadiusMm;
    headMetrics[QStringLiteral("containerIcpConverged")] = measurement.containerIcpConverged;
    if (!measurement.codeValue.isEmpty()) {
        headMetrics[QStringLiteral("codeValue")] = measurement.codeValue;
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
        appendKeyValue(&text, QStringLiteral("lengthMm"), m.lengthMm);
        appendKeyValue(&text, QStringLiteral("volumeLiters"), m.volumeLiters);
        appendKeyValue(&text, QStringLiteral("volumeRadiusMm"), m.volumeRadiusMm);
        appendKeyValue(&text, QStringLiteral("fittedOuterRadiusMm"), m.fittedOuterRadiusMm);
        appendKeyValue(&text, QStringLiteral("containerLeftEndPositionMm"),
                       m.containerLeftEndPositionMm);
        appendKeyValue(&text, QStringLiteral("containerRightEndPositionMm"),
                       m.containerRightEndPositionMm);
        appendKeyValue(&text, QStringLiteral("containerIcpFitness"), m.containerIcpFitness);
        appendKeyValue(&text, QStringLiteral("containerFittedRadiusMm"),
                       m.containerFittedRadiusMm);
        appendKeyValue(&text, QStringLiteral("containerIcpConverged"),
                       static_cast<qint64>(m.containerIcpConverged ? 1 : 0));
        appendKeyValue(&text, QStringLiteral("measuredSegmentCount"),
                       static_cast<qint64>(m.measuredSegmentCount));
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
    QFile file(filePath);
    const bool needsHeader = !file.exists() || file.size() == 0;
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("无法打开 %1：%2")
                                .arg(filePath, file.errorString());
        }
        return false;
    }

    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    if (needsHeader) {
        const auto& m = result.measurement;
        stream << QStringLiteral("测试目标指标（12项，左右侧咬边合并计项）:\n")
               << QStringLiteral("1. 焊缝错边量 = ") << m.mismatchMm << QStringLiteral(" mm\n")
               << QStringLiteral("2. 焊缝余高 = ") << m.reinforcementMm << QStringLiteral(" mm\n")
               << QStringLiteral("3. 母材拟合线夹角 = ") << m.includedAngleDeg << QStringLiteral(" deg\n")
               << QStringLiteral("4. 棱角度 = ") << m.angularityMm << QStringLiteral(" mm\n")
               << QStringLiteral("5. 左右咬边深度 = 左 ") << m.leftUndercutMm
               << QStringLiteral(" mm，右 ") << m.rightUndercutMm << QStringLiteral(" mm\n")
               << QStringLiteral("6. 左右咬边长度 = 左 ") << m.leftUndercutLengthMm
               << QStringLiteral(" mm，右 ") << m.rightUndercutLengthMm << QStringLiteral(" mm\n")
               << QStringLiteral("7. 筒体焊缝附近平均厚度 = ") << m.thicknessMm << QStringLiteral(" mm\n")
               << QStringLiteral("8. 内径 = ") << m.innerDiameterMm << QStringLiteral(" mm\n")
               << QStringLiteral("9. 内周长 = ") << m.innerCircumferenceMm << QStringLiteral(" mm\n")
               << QStringLiteral("10. 内表面圆度 = ") << m.innerRoundness << QStringLiteral(" mm\n")
               << QStringLiteral("11. 筒体总长 = ") << m.lengthMm << QStringLiteral(" mm\n")
               << QStringLiteral("12. 容积 = ") << m.volumeLiters << QStringLiteral(" L\n\n");
    }
    stream << formatInspectionResultTextBlock(result);
    stream.flush();
    if (stream.status() != QTextStream::Ok) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("写入 %1 失败。").arg(filePath);
        }
        return false;
    }
    return true;
}

}  // namespace flow_control
}  // namespace scan_tracking
