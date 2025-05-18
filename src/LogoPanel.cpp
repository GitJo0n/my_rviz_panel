#include <rviz/panel.h> // ROS 1 rviz
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QFrame> // QFrame 추가
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace my_rviz_panel
{
  class LogoPanel : public rviz::Panel 
  {
    Q_OBJECT
  public:
    LogoPanel(QWidget* parent = 0)
      : rviz::Panel(parent) 
    {
      // 콘텐츠를 담을 QFrame 생성
      QFrame* content_frame = new QFrame(this);
      content_frame->setObjectName("contentFrame"); // QSS에서 선택자로 사용하기 좋음
      content_frame->setStyleSheet(
        "QFrame#contentFrame {"
        "  border: 4px solid orange;"    // 3픽셀 보라색 실선 테두리
        "  border-radius: 8px;"         // 모서리 둥글게
        "  background-color: white;"     // 프레임 배경색
        "  padding: 10px;"               // 프레임 내부 여백
        "}"
      );

      // QLabel은 content_frame의 자식으로 생성
      label_ = new QLabel(content_frame);
      QPixmap pixmap("/home/user/catkin_ws/src/my_rviz_panel/resources/logo.png");
      if(pixmap.isNull()){
        ROS_WARN("LogoPanel: Failed to load logo.png. Check path.");
        label_->setText("Logo not found");
      } else {
        label_->setPixmap(pixmap);
      }
      label_->setAlignment(Qt::AlignCenter);

      // QFrame 내부 레이아웃 설정
      QVBoxLayout* frame_layout = new QVBoxLayout(content_frame);
      frame_layout->addWidget(label_);
      // content_frame->setLayout(frame_layout); // 생성자에 부모 위젯을 전달하면 자동 설정됨

      // 패널의 메인 레이아웃 설정 (QFrame을 패널에 추가)
      QVBoxLayout* panel_layout = new QVBoxLayout(this);
      panel_layout->addWidget(content_frame);
      setLayout(panel_layout);

      ros::NodeHandle nh;
      subscriber_ = nh.subscribe("/status_text", 10, &LogoPanel::updateText, this);
    }

  private Q_SLOTS:
    void updateText(const std_msgs::String::ConstPtr& msg)
    {
      label_->setText(QString::fromStdString(msg->data));
    }

  private:
    QLabel* label_;
    ros::Subscriber subscriber_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::LogoPanel, rviz::Panel) // 또는 rviz::Panel
#include "LogoPanel.moc"
