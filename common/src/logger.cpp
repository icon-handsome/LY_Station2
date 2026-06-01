#include "scan_tracking/common/logger.h"

#include <QtCore/QDateTime>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <system_error>

namespace scan_tracking::common {

namespace {

constexpr char kUtf8Bom[] = "\xEF\xBB\xBF";

thread_local int g_log_handler_depth = 0;

const char* safeCategoryName(const QMessageLogContext& context)
{
    if (context.category != nullptr && context.category[0] != '\0') {
        return context.category;
    }
    return "default";
}

std::string dailyLogFilePath(const std::string& log_dir, const QDate& date)
{
    return log_dir + "/scan_tracking_"
        + date.toString(QStringLiteral("yyyy-MM-dd")).toStdString() + ".txt";
}

bool ensureLogDirectory(const std::string& log_dir)
{
    if (log_dir.empty()) {
        return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(log_dir, ec);
    if (ec) {
        return false;
    }
    return std::filesystem::is_directory(log_dir);
}

void writeConsoleLine(QtMsgType type, const char* data, std::size_t size)
{
    FILE* out = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg) ? stderr : stdout;
    std::fwrite(data, 1, size, out);
    std::fwrite("\n", 1, 1, out);
    std::fflush(out);
}

std::string buildLogLine(
    const char* time_stamp,
    const char* severity,
    const char* category,
    const char* msg_utf8,
    const char* source_suffix)
{
    std::string line;
    line.reserve(
        std::strlen(time_stamp) + std::strlen(severity) + std::strlen(category)
        + std::strlen(msg_utf8) + (source_suffix != nullptr ? std::strlen(source_suffix) : 0) + 32);
    line += '[';
    line += time_stamp;
    line += "] [";
    line += severity;
    line += "] [";
    line += category;
    line += "] ";
    line += msg_utf8;
    if (source_suffix != nullptr && source_suffix[0] != '\0') {
        line += source_suffix;
    }
    return line;
}

std::string sourceLocationSuffix(const QMessageLogContext& context)
{
    if (context.file == nullptr || context.line <= 0) {
        return {};
    }

    std::string suffix = " (";
    suffix += context.file;
    suffix += ':';
    suffix += std::to_string(context.line);
    suffix += ')';
    return suffix;
}

void emitMinimalFallback(QtMsgType type, const QMessageLogContext& context, const QByteArray& msg_utf8)
{
    const char* severity = "UNK";
    switch (type) {
        case QtDebugMsg: severity = "DBG"; break;
        case QtInfoMsg: severity = "INF"; break;
        case QtWarningMsg: severity = "WRN"; break;
        case QtCriticalMsg: severity = "CRT"; break;
        case QtFatalMsg: severity = "FTL"; break;
        default: break;
    }
    const std::string suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : std::string{};
    const std::string line = buildLogLine(
        "reentrant",
        severity,
        safeCategoryName(context),
        msg_utf8.constData(),
        suffix.empty() ? nullptr : suffix.c_str());
    writeConsoleLine(type, line.c_str(), line.size());
}

}  // namespace

Logger* Logger::instance_ = nullptr;
QtMessageHandler Logger::previous_handler_ = nullptr;

Logger::Logger(const QString& log_dir)
    : log_dir_(log_dir.toStdString()),
      min_level_(QtDebugMsg)
{
    if (!ensureLogDirectory(log_dir_)) {
        std::cerr << "严重错误：Logger 无法创建日志目录：" << log_dir_ << "\n";
    }
    openLogFile(QDate::currentDate());
}

Logger::~Logger()
{
    if (log_file_ != nullptr) {
        std::fclose(log_file_);
        log_file_ = nullptr;
    }
}

void Logger::initialize(const QString& log_dir)
{
    if (instance_ != nullptr) {
        return;
    }

    instance_ = new Logger(log_dir);
    previous_handler_ = qInstallMessageHandler(Logger::messageHandler);
}

void Logger::cleanup()
{
    if (instance_ == nullptr) {
        return;
    }

    QtMessageHandler upstream = previous_handler_;
    qInstallMessageHandler(upstream);
    previous_handler_ = nullptr;

    Logger* doomed = instance_;
    instance_ = nullptr;

    {
        std::lock_guard<std::mutex> lock(doomed->mutex_);
    }
    delete doomed;
}

Logger* Logger::instance()
{
    return instance_;
}

void Logger::setMinLevel(QtMsgType level)
{
    std::lock_guard<std::mutex> lock(mutex_);
    min_level_ = level;
}

int Logger::getSeverityLevel(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return 0;
        case QtInfoMsg: return 1;
        case QtWarningMsg: return 2;
        case QtCriticalMsg: return 3;
        case QtFatalMsg: return 4;
        default: return 0;
    }
}

const char* Logger::getLogSeverity(QtMsgType type)
{
    switch (type) {
        case QtDebugMsg: return "DBG";
        case QtInfoMsg: return "INF";
        case QtWarningMsg: return "WRN";
        case QtCriticalMsg: return "CRT";
        case QtFatalMsg: return "FTL";
        default: return "UNK";
    }
}

void Logger::openLogFile(const QDate& target_date)
{
    if (!target_date.isValid()) {
        return;
    }

    const std::string file_path = dailyLogFilePath(log_dir_, target_date);

    if (log_file_ != nullptr) {
        std::fclose(log_file_);
        log_file_ = nullptr;
    }

    log_file_ = std::fopen(file_path.c_str(), "ab");
    if (log_file_ == nullptr) {
        std::cerr << "严重错误：Logger 无法打开目标文件：" << file_path << "\n";
        return;
    }

    std::fseek(log_file_, 0, SEEK_END);
    if (std::ftell(log_file_) == 0) {
        std::fwrite(kUtf8Bom, 1, 3, log_file_);
        std::fflush(log_file_);
    }

    current_date_ = target_date;
}

void Logger::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    Logger* logger = instance_;
    if (logger != nullptr) {
        logger->log(type, context, msg);
        return;
    }
    if (previous_handler_ != nullptr) {
        previous_handler_(type, context, msg);
    }
}

void Logger::log(QtMsgType type, const QMessageLogContext& context, const QString& msg)
{
    const QByteArray msg_utf8 = msg.toUtf8();

    struct DepthGuard {
        DepthGuard() { ++g_log_handler_depth; }
        ~DepthGuard() { --g_log_handler_depth; }
    } depth_guard;

    if (g_log_handler_depth > 1) {
        emitMinimalFallback(type, context, msg_utf8);
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (getSeverityLevel(type) < getSeverityLevel(min_level_)) {
        return;
    }

    const QDateTime now = QDateTime::currentDateTime();
    if (now.date() != current_date_) {
        openLogFile(now.date());
    }

    const std::string time_stamp = now.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz")).toStdString();
    const std::string suffix = (type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg)
        ? sourceLocationSuffix(context)
        : std::string{};
    const std::string line = buildLogLine(
        time_stamp.c_str(),
        getLogSeverity(type),
        safeCategoryName(context),
        msg_utf8.constData(),
        suffix.empty() ? nullptr : suffix.c_str());

    if (log_file_ != nullptr) {
        std::fwrite(line.c_str(), 1, line.size(), log_file_);
        std::fwrite("\r\n", 1, 2, log_file_);
        std::fflush(log_file_);
    }

    writeConsoleLine(type, line.c_str(), line.size());
}

}  // namespace scan_tracking::common
