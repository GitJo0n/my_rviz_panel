#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QDateTime>
#include <QString>
#include <pluginlib/class_list_macros.h>

namespace rviz_custom_panels
{

class RealTimeClockPanel : public rviz::Panel
{
  Q_OBJECT

public:
  RealTimeClockPanel(QWidget* parent = 0)
    : rviz::Panel(parent)
  {
    time_label_ = new QLabel(this);
    time_label_->setAlignment(Qt::AlignCenter);
    time_label_->setStyleSheet("QLabel { font-size: 14pt; color: white; background-color: #333333; padding: 5px; }");

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(time_label_);
    setLayout(layout);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &RealTimeClockPanel::updateDisplayTime);
    timer_->start(1000); // 1초마다 업데이트

    updateDisplayTime(); // 초기 시간 표시
  }

private Q_SLOTS:
  void updateDisplayTime()
  {
    // 현재 날짜 및 시간 (2025년 5월 18일 기준)
    QDateTime current_time = QDateTime::currentDateTime();
    QString time_string = current_time.toString("yyyy-MM-dd hh:mm:ss");
    time_label_->setText(time_string);
  }

private:
  QLabel* time_label_;
  QTimer* timer_;
};

} // namespace rviz_custom_panels

PLUGINLIB_EXPORT_CLASS(rviz_custom_panels::RealTimeClockPanel, rviz::Panel)

// CMake에서 CMAKE_AUTOMOC ON으로 설정된 경우 아래 MOC 파일 include는 필요 없을 수 있습니다.
// 만약 필요하다면, 이 파일의 이름을 RealTimeClockPanel.cpp로 가정했을 때,
// 빌드 시스템은 RealTimeClockPanel.moc 파일을 생성하여 링크합니다.
#include "RealTimeClockPanel.moc"