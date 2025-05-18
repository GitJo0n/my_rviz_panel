#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QString>
#include <QFrame> // QFrame 헤더 추가
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

namespace my_rviz_panel
{
  class KeyboardPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    KeyboardPanel(QWidget* parent = 0)
      : rviz::Panel(parent)
    {
      pkg_path_ = ros::package::getPath("my_rviz_panel");
      if (pkg_path_.empty()) {
        ROS_ERROR("KeyboardPanel: Could not find package my_rviz_panel. Ensure it's in your ROS_PACKAGE_PATH.");
      }

      QFrame* content_frame = new QFrame(this); 
      content_frame->setObjectName("keyboardContentFrame"); 
      content_frame->setStyleSheet(
        "QFrame#keyboardContentFrame {"
        "  border: 4px solid orange;"    // 주황색 테두리
        "  border-radius: 8px;"         // 모서리 둥글게
        "  background-color: white;"     // 프레임 배경색
        "  padding: 10px;"               // 프레임 내부 여백
        "}"
      );

      // QLabel은 content_frame의 자식으로 생성
      direction_label_ = new QLabel(content_frame);
      std::string default_image_path = pkg_path_ + "/resources/idle.png";
      QPixmap default_pixmap(QString::fromStdString(default_image_path));
      if(default_pixmap.isNull()){
          ROS_WARN("KeyboardPanel: Failed to load default image: %s", default_image_path.c_str());
          direction_label_->setText("idle.png not found");
      } else {
          direction_label_->setPixmap(default_pixmap);
      }
      direction_label_->setAlignment(Qt::AlignCenter);
      direction_label_->setScaledContents(true); // 이미지 크기를 QLabel 크기에 맞춤
      direction_label_->setFixedSize(400, 200);  // QLabel 크기 고정

      // QFrame 내부 레이아웃 설정
      QVBoxLayout* frame_layout = new QVBoxLayout(content_frame); 
      frame_layout->addWidget(direction_label_);
      // content_frame->setLayout(frame_layout); // QVBoxLayout 생성자에 부모 위젯(content_frame)을 전달하면 자동 설정됨

      // 패널의 메인 레이아웃 설정 (QFrame을 패널에 추가)
      QVBoxLayout* panel_layout = new QVBoxLayout(this); 
      panel_layout->addWidget(content_frame);
      setLayout(panel_layout);

      ros::NodeHandle nh;
      subscriber_key_ = nh.subscribe("/keyboard_input", 10, &KeyboardPanel::updateDirectionImage, this);
    }

  private Q_SLOTS:
    void updateDirectionImage(const std_msgs::String::ConstPtr& msg)
    {
        std::string image_file;

        if (msg->data == "w") image_file = "/resources/forward.png";
        else if (msg->data == "s") image_file = "/resources/backward.png";
        else if (msg->data == "a") image_file = "/resources/left.png";
        else if (msg->data == "d") image_file = "/resources/right.png";
        else image_file = "/resources/idle.png";

        if (pkg_path_.empty()) {
            ROS_ERROR_THROTTLE(5.0, "KeyboardPanel: Package path for my_rviz_panel is not set. Cannot load images.");
            direction_label_->setText("Error: pkg_path not set");
            return;
        }

        std::string full_path = pkg_path_ + image_file;
        QPixmap pixmap(QString::fromStdString(full_path));

        if(pixmap.isNull()){
            ROS_WARN_THROTTLE(2.0, "KeyboardPanel: Failed to load image: %s", full_path.c_str());
            direction_label_->setText(QString("Img not found:\n%1").arg(QString::fromStdString(image_file).split('/').last()));
        } else {
            direction_label_->setPixmap(pixmap);
        }
    }

  private:
    QLabel* direction_label_;
    ros::Subscriber subscriber_key_;
    std::string pkg_path_;
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::KeyboardPanel, rviz::Panel)
#include "KeyboardPanel.moc"