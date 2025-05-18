#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QString>
#include <QFrame>
#include <QTimer> // QTimer 헤더 추가
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

namespace my_rviz_panel
{
  class Control_Keyboard : public rviz::Panel
  {
    Q_OBJECT
  public:
    Control_Keyboard(QWidget* parent = 0)
      : rviz::Panel(parent)
    {
      pkg_path_ = ros::package::getPath("my_rviz_panel");
      if (pkg_path_.empty()) {
        ROS_ERROR("Control_Keyboard: Could not find package my_rviz_panel. Ensure it's in your ROS_PACKAGE_PATH.");
      }

      QFrame* content_frame = new QFrame(this);
      content_frame->setObjectName("keyboardContentFrame");
      content_frame->setStyleSheet(
        "QFrame#keyboardContentFrame {"
        "  border: 4px solid orange;"
        "  border-radius: 8px;"
        "  background-color: white;"
        "  padding: 10px;"
        "}"
      );

      direction_label_ = new QLabel(content_frame);
      direction_label_->setAlignment(Qt::AlignCenter);
      direction_label_->setScaledContents(true);
      direction_label_->setFixedSize(400, 200);

      QVBoxLayout* frame_layout = new QVBoxLayout(content_frame);
      frame_layout->addWidget(direction_label_);

      QVBoxLayout* panel_layout = new QVBoxLayout(this);
      panel_layout->addWidget(content_frame);
      setLayout(panel_layout);

      // Idle 상태로 돌아가기 위한 타이머 설정
      idle_timer_ = new QTimer(this);
      idle_timer_->setSingleShot(true); // 타이머가 한 번만 실행되도록 설정
      connect(idle_timer_, &QTimer::timeout, this, &Control_Keyboard::showIdleImage);

      ros::NodeHandle nh;
      subscriber_key_ = nh.subscribe("/keyboard_input", 10, &Control_Keyboard::keyboardCallback, this);

      // 초기 이미지를 idle 상태로 설정
      showIdleImage();
    }

  private Q_SLOTS:
    void showIdleImage()
    {
      if (pkg_path_.empty()) {
        ROS_ERROR_THROTTLE(5.0, "Control_Keyboard: Package path for my_rviz_panel is not set. Cannot load idle image.");
        direction_label_->setText("Error: pkg_path not set\nCannot load idle.png");
        return;
      }
      std::string idle_image_path = pkg_path_ + "/resources/idle.png";
      QPixmap pixmap(QString::fromStdString(idle_image_path));

      if(pixmap.isNull()){
          ROS_WARN("Control_Keyboard: Failed to load idle image: %s", idle_image_path.c_str());
          direction_label_->setText("idle.png not found");
      } else {
          direction_label_->setPixmap(pixmap);
      }
    }

    void keyboardCallback(const std_msgs::String::ConstPtr& msg)
    {
        // 새로운 키 입력이 들어오면 기존 타이머를 중지
        idle_timer_->stop();

        std::string image_file_suffix;
        bool is_known_key = true; // 인식된 키인지 여부

        if (msg->data == "w") image_file_suffix = "/resources/forward.png";
        else if (msg->data == "s") image_file_suffix = "/resources/backward.png";
        else if (msg->data == "a") image_file_suffix = "/resources/left.png";
        else if (msg->data == "d") image_file_suffix = "/resources/right.png";
        else {
            is_known_key = false; //idle 상태
        }

        if (pkg_path_.empty()) {
            ROS_ERROR_THROTTLE(5.0, "Control_Keyboard: Package path for my_rviz_panel is not set. Cannot load images.");
            direction_label_->setText("Error: pkg_path not set");
            if(is_known_key) { 
                 idle_timer_->start(200); // 200 밀리초 후 idle 상태
            } else {
                showIdleImage(); // 인식되지 않은 키면 바로 idle
            }
            return;
        }

        if (!is_known_key) { // w, a, s, d 가 아닌 경우 (또는 명시적으로 idle로 가야하는 경우)
            showIdleImage();
            return; 
        }

        std::string full_path = pkg_path_ + image_file_suffix;
        QPixmap pixmap(QString::fromStdString(full_path));

        if(pixmap.isNull()){
            ROS_WARN_THROTTLE(2.0, "Control_Keyboard: Failed to load image: %s", full_path.c_str());
            direction_label_->setText(QString("Img not found:\n%1").arg(QString::fromStdString(image_file_suffix).split('/').last()));
        } else {
            direction_label_->setPixmap(pixmap);
        }

        idle_timer_->start(200); // 200 밀리초 후에 idle 상태로 (시간은 조절 가능)
                               
    }

  private:
    QLabel* direction_label_;
    ros::Subscriber subscriber_key_;
    std::string pkg_path_;
    QTimer* idle_timer_; // idle 상태로 돌아가기 위한 타이머
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::Control_Keyboard, rviz::Panel)
#include "Control_Keyboard.moc"