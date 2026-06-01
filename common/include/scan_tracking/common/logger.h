#pragma once

#include <QtCore/QString>
#include <QtCore/QtMessageHandler>
#include <QtCore/QDate>

#include <cstdio>
#include <mutex>

namespace scan_tracking::common {

// 按自然日各写一个 txt：logs/scan_tracking_yyyy-MM-dd.txt；跨日自动切换文件，仅追加、不覆盖历史。
class Logger {
public:
    static void initialize(const QString& log_dir = QStringLiteral("logs"));
    static void cleanup();

    static void messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static Logger* instance();

    void setMinLevel(QtMsgType level);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    explicit Logger(const QString& log_dir);
    ~Logger();

    void openLogFile(const QDate& target_date);
    void log(QtMsgType type, const QMessageLogContext& context, const QString& msg);

    static const char* getLogSeverity(QtMsgType type);
    static int getSeverityLevel(QtMsgType type);

    std::string log_dir_;
    FILE* log_file_ = nullptr;
    std::mutex mutex_;
    QDate current_date_;
    QtMsgType min_level_;

    static Logger* instance_;
    static QtMessageHandler previous_handler_;
};

}  // namespace scan_tracking::common
