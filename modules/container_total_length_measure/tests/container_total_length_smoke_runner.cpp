#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <cstdio>
#include <vector>

#include "scan_tracking/container_total_length_measure/container_total_length_service.h"
#include "scan_tracking/container_total_length_measure/container_total_length_worker_protocol.h"

using scan_tracking::container_total_length_measure::ContainerTotalLengthError;
using scan_tracking::container_total_length_measure::ContainerTotalLengthMeasurement;
using scan_tracking::container_total_length_measure::ContainerTotalLengthService;
using scan_tracking::container_total_length_measure::worker_protocol::kWorkerRelPath;

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    const QString workerPath =
        QDir(QCoreApplication::applicationDirPath()).filePath(QString::fromLatin1(kWorkerRelPath));
    if (!QFileInfo::exists(workerPath)) {
        std::fprintf(stderr, "FAIL: worker missing: %s\n", qPrintable(workerPath));
        return 2;
    }
    std::fprintf(stdout, "OK: worker present: %s\n", qPrintable(workerPath));

    ContainerTotalLengthService service;
    ContainerTotalLengthError error;
    if (!service.initializeFromIni(QString(), &error)) {
        std::fprintf(stderr, "FAIL: initializeFromIni: %s\n", qPrintable(error.message));
        return 3;
    }
    std::fprintf(stdout, "OK: initializeFromIni config=%s\n", qPrintable(service.configPath()));

    // Round-1 smoke: prove OOP path can start the worker and return a structured result.
    // A tiny synthetic cloud is expected to fail measurement; missing worker / crash is not.
    std::vector<float> xyz;
    xyz.reserve(30 * 3);
    for (int i = 0; i < 30; ++i) {
        xyz.push_back(static_cast<float>(i));
        xyz.push_back(0.0f);
        xyz.push_back(0.0f);
    }

    ContainerTotalLengthMeasurement measurement;
    const bool ok = service.measure(xyz.data(), xyz.size() / 3, &measurement, &error);
    if (ok) {
        std::fprintf(
            stdout,
            "OK: measure succeeded unexpectedly on synthetic cloud lengthMm=%.3f\n",
            measurement.lengthMm);
        return 0;
    }

    // Worker reached and answered (status from result.txt) vs host could not launch worker.
    if (error.statusCode == 100 || error.statusCode == 101 || error.statusCode == 102) {
        std::fprintf(
            stderr,
            "FAIL: worker infrastructure error status=%d message=%s\n",
            error.statusCode,
            qPrintable(error.message));
        return 4;
    }

    std::fprintf(
        stdout,
        "OK: worker responded with measure failure (expected for synthetic cloud) "
        "status=%d message=%s\n",
        error.statusCode,
        qPrintable(error.message));
    return 0;
}
