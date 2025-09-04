#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QDateTime>
#include <QString>
#include <QFrame> // QFrame 사용을 위해 추가
#include <QTimeZone>
#include <pluginlib/class_list_macros.h>

namespace my_rviz_panel
{

class RealTimeClockPanel : public rviz::Panel
{
  Q_OBJECT

public:
  RealTimeClockPanel(QWidget* parent = 0)
    : rviz::Panel(parent)
  {
    QFrame* content_frame = new QFrame(this);
    content_frame->setObjectName("clockContentFrame"); 
    content_frame->setStyleSheet(
        "QFrame#clockContentFrame {"
        "  border: 4px solid orange;"    // 이전과 동일한 주황색 테두리
        "  border-radius: 8px;"         // 모서리 둥글게
        "  background-color: white;"     // 프레임 배경색 (time_label_의 배경과 구분되도록)
        "  padding: 10px;"               // 프레임 안쪽 여백
        "}"
    );

    time_label_ = new QLabel(content_frame);
    time_label_->setAlignment(Qt::AlignCenter);
    time_label_->setStyleSheet("QLabel { font-size: 14pt; color: black; background-color:rgb(255, 255, 255); padding: 5px; border-radius: 4px; }"); // 약간의 radius 추가

    QVBoxLayout* frame_layout = new QVBoxLayout(content_frame); 
    frame_layout->addWidget(time_label_);

    QVBoxLayout* panel_layout = new QVBoxLayout(this); 
    panel_layout->addWidget(content_frame);
    setLayout(panel_layout);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &RealTimeClockPanel::updateDisplayTime);
    timer_->start(1000); // 1초마다 업데이트

    updateDisplayTime(); 
  }

private Q_SLOTS:
void updateDisplayTime()
{
  // UTC 시간을 기준으로 한국 시간대
  QDateTime current_time = QDateTime::currentDateTime().toTimeZone(QTimeZone("Asia/Seoul"));

  // 시간 형식을 24시간제('HH')
  QString time_string = current_time.toString("yyyy-MM-dd HH:mm:ss");
  time_label_->setText(time_string);
}

private:
  QLabel* time_label_;
  QTimer* timer_;
};

} 

PLUGINLIB_EXPORT_CLASS(my_rviz_panel::RealTimeClockPanel, rviz::Panel)

#include "RealTimeClockPanel.moc"