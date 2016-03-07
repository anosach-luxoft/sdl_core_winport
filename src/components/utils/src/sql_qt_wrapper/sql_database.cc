#include "sql_qt_wrapper/sql_database.h"

#include <QSqlError>

#include "utils/string_utils.h"

namespace {
const size_t kConnectionNameLength = 5;
}  // namespace

namespace utils {
namespace dbms {

SQLDatabase::SQLDatabase(const std::string& filename)
    : databasename_(filename)
    , connection_name_(utils::GenerateRandomString(kConnectionNameLength)) {
  db_ = QSqlDatabase::addDatabase(QString("QSQLITE"),
                                  QString(connection_name_.c_str()));
}

SQLDatabase::~SQLDatabase() {
  Close();
  QSqlDatabase::removeDatabase(QString(connection_name_.c_str()));
}

bool SQLDatabase::Open() {
  db_.setDatabaseName(databasename_.c_str());
  return db_.open();
}

void SQLDatabase::Close() {
  db_.close();
}

bool SQLDatabase::BeginTransaction() {
  return db_.transaction();
}

bool SQLDatabase::CommitTransaction() {
  return db_.commit();
}

bool SQLDatabase::RollbackTransaction() {
  return db_.rollback();
}

SQLError SQLDatabase::LastError() const {
  return SQLError(db_.lastError());
}

bool SQLDatabase::HasErrors() const {
  return db_.lastError().type() != QSqlError::NoError;
}

std::string SQLDatabase::get_path() const {
  return databasename_;
}

bool SQLDatabase::IsReadWrite() {
  return true;
}

bool SQLDatabase::Backup() {
  return true;
}

SQLDatabase::operator QSqlDatabase() const {
  return db_;
}

bool SQLDatabase::Exec(const std::string& query) {
  return true;
}

}  // namespace dbms
}  // namespace utils
