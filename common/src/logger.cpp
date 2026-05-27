#include "scan_tracking/common/logger.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QMutexLocker>
#include <cstdio>
#include <iostream>
namespace scan_tracking::common {

namespace {

constexpr char kUtf8Bom[] = "\xEF\xBB\xBF";

QString dailyLogFilePath(const QString& log_dir, const QDate& date)
{
    return QDir(log_dir).filePath(
        QStringLiteral("scan_tracking_%1.txt").arg(date.toString(QStringLiteral("yyyy-MM-dd"))));
}

bool hasUtf8Bom(const QByteArray& data)
{
    return data.size() >= 3
        && static_cast<unsigned char>(data.at(0)) == 0xEF
        && static_cast<unsigned char>(data.at(1)) == 0xBB
        && static_cast<unsigned char>(data.at(2)) == 0xBF;
}

struct LogFilePayload {
    QByteArray content;
    bool hadLeadingNullPadding = false;
};

LogFilePayload loadLogFilePayload(const QString& file_path)
{
    LogFilePayload payload;
    QFile in(file_path);
    if (!in.exists() || !in.open(QIODevice::ReadOnly)) {
        return payload;
    }

    const QByteArray raw = in.readAll();
    in.close();

    int start = 0;
    while (start < raw.size() && raw.at(start) == '\0') {
        ++start;
    }
    payload.hadLeadingNullPadding = start > 0 && start < raw.size();
    payload.content = payload.hadLeadingNullPadding ? raw.mid(start) : raw;
    return payload;
}

bool rewriteLogFileWithUtf8Bom(const QString& file_path, const QByteArray& content)
{
    QFile out(file_path);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }

    if (!hasUtf8Bom(content)) {
        out.write(kUtf8Bom);
    }
    if (!content.isEmpty()) {
        out.write(content);
    }
    out.close();
    return true;
}

}  // namespace

Logger* Logger::instance_ = nullptr;
QtMessageHandler Logger::previous_handler_ = nullptr;

Logger::Logger(const QString& log_dir)
    : log_dir_(log_dir),
      log_file_(new QFile()),
      min_level_(QtDebugMsg) {
    QDir dir(log_dir_);
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    openLogFile(QDate::currentDate());
}

Logger::~Logger() {
    if (log_file_) {
        if (log_file_->isOpen()) {
            log_file_->close();
        }
        delete log_file_;
        log_file_ = nullptr;
    }
}

void Logger::initialize(const QString& log_dir) {
    if (!instance_) {
        instance_ = new Logger(log_dir);
        previous_handler_ = qInstallMessageHandler(Logger::messageHandler);
    }
}

void Logger::cleanup() {
    if (!instance_) {
        return;
    }

    // 先恢复上游 handler，避免析构过程中仍有线程进入已释放的 Logger
    QtMessageHandler upstream = previous_handler_;
    qInstallMessageHandler(upstream);
    previous_handler_ = nullptr;

    Logger* doomed = instance_;
    instance_ = nullptr;

    {
        QMutexLocker locker(&doomed->mutex_);
    }
    delete doomed;
}

Logger* Logger::instance() {
    return instance_;
}

void Logger::setMinLevel(QtMsgType level) {
    QMutexLocker locker(&mutex_);
    min_level_ = level;
}

int Logger::getSeverityLevel(QtMsgType type) {
    switch (type) {
        case QtDebugMsg:    return 0;
        case QtInfoMsg:     return 1;
        case QtWarningMsg:  return 2;
        case QtCriticalMsg: return 3;
        case QtFatalMsg:    return 4;
        default:            return 0;
    }
}

QString Logger::getLogSeverity(QtMsgType type) {
    switch (type) {
        case QtDebugMsg:    return QStringLiteral("DBG");
        case QtInfoMsg:     return QStringLiteral("INF");
        case QtWarningMsg:  return QStringLiteral("WRN");
        case QtCriticalMsg: return QStringLiteral("CRT");
        case QtFatalMsg:    return QStringLiteral("FTL");
        default:            return QStringLiteral("UNK");
    }
}

void Logger::openLogFile(const QDate& target_date) {
    // 二进制追加 + 手写 UTF-8，避免 Windows Text 模式与编码混用；同日重启仅追加不截断。
    static const QIODevice::OpenMode kLogOpenMode = QIODevice::WriteOnly | QIODevice::Append;

    if (!target_date.isValid()) {
        return;
    }

    const QString file_path = dailyLogFilePath(log_dir_, target_date);

    const QString previous_path = log_file_->fileName();
    const bool was_open = log_file_->isOpen();
    if (was_open) {
        log_file_->close();
    }

    const LogFilePayload existing = loadLogFilePayload(file_path);
    if (existing.hadLeadingNullPadding
        || (!existing.content.isEmpty() && !hasUtf8Bom(existing.content))) {
        if (!rewriteLogFileWithUtf8Bom(file_path, existing.content)) {
            std::cerr << "严重错误：Logger 无法规范化日志文件：" << file_path.toStdString()
                      << "\n";
        }
    }

    log_file_->setFileName(file_path);
    if (!log_file_->open(kLogOpenMode)) {
        std::cerr << "严重错误：Logger 无法打开目标文件：" << file_path.toStdString() << "\n";
        log_file_->setFileName(previous_path);
        if (was_open && !previous_path.isEmpty()) {
            if (!log_file_->open(kLogOpenMode)) {
                std::cerr << "严重错误：Logger 无法恢复上一日志文件：" << previous_path.toStdString()
                          << "\n";
            }
        }
        return;
    }

    if (log_file_->size() == 0) {
        log_file_->write(kUtf8Bom);
        log_file_->flush();
    }

    current_date_ = target_date;
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    Logger* logger = instance_;
    if (logger) {
        logger->log(type, context, msg);
        return;
    }
    if (previous_handler_) {
        previous_handler_(type, context, msg);
    }
}

void Logger::log(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    QMutexLocker locker(&mutex_);

    if (getSeverityLevel(type) < getSeverityLevel(min_level_)) {
        return;
    }

    QDateTime now = QDateTime::currentDateTime();
    
    if (now.date() != current_date_) {
        openLogFile(now.date());
    }

    QString time_stamp = now.toString("yyyy-MM-dd HH:mm:ss.zzz");
    QString severity = getLogSeverity(type);
    QString category = QString::fromLatin1(context.category ? context.category : "default");
    
    QString formatted_message = QStringLiteral("[%1] [%2] [%3] %4").arg(time_stamp, severity, category, msg);

    if (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) {
        if (context.file && context.line > 0) {
            formatted_message.append(QStringLiteral(" (%1:%2)").arg(QString::fromLatin1(context.file)).arg(context.line));
        }
    }

    const QByteArray utf8FileLine = formatted_message.toUtf8() + "\r\n";
    const QByteArray utf8ConsoleLine = formatted_message.toUtf8() + '\n';
    if (log_file_ && log_file_->isOpen()) {
        log_file_->write(utf8FileLine);
        log_file_->flush();
    }

    // 避免 QTextStream+iostream 混用导致 Debug 堆损坏；控制台仅写 UTF-8 行
    FILE* out = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) ? stderr : stdout;
    std::fwrite(utf8ConsoleLine.constData(), 1, static_cast<size_t>(utf8ConsoleLine.size()), out);
    std::fflush(out);
}

} // namespace scan_tracking::common
