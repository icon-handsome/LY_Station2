#include <QtCore/QCoreApplication>
#include <QtCore/QLoggingCategory>
#include <QtCore/QString>
#include <QtCore/QTextStream>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_worker.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace {

void printUsage()
{
    qInfo(appLog).noquote()
        << "Usage: gemini335_smoke_runner [--serial SN | --index N]\n"
        << "  --serial SN   Open device by serial number\n"
        << "  --index N     Open device by index (default 0)\n"
        << "  -h, --help    Show this help";
}

bool parseArgs(const QStringList& args, scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig* config)
{
    for (int index = 1; index < args.size(); ++index) {
        const QString& token = args.at(index);
        if (token == QStringLiteral("-h") || token == QStringLiteral("--help")) {
            printUsage();
            return false;
        }
        if (token == QStringLiteral("--serial") && index + 1 < args.size()) {
            config->serial = args.at(++index);
            continue;
        }
        if (token == QStringLiteral("--index") && index + 1 < args.size()) {
            config->deviceIndex = args.at(++index).toInt();
            continue;
        }
    }

    return true;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("gemini335_smoke_runner"));

    scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig config;
    if (!parseArgs(QCoreApplication::arguments(), &config)) {
        return 0;
    }

    scan_tracking::orbbec_gemini::OrbbecGeminiWorker worker;
    QObject::connect(
        &worker,
        &scan_tracking::orbbec_gemini::OrbbecGeminiWorker::logMessage,
        [](const QString& message) {
            qInfo(appLog).noquote() << message;
            QTextStream stream(stdout);
            stream << message << '\n';
            stream.flush();
        });

    worker.startWorker(config);
    worker.stopWorker();
    return 0;
}
