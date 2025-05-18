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
        // 패키지 경로가 없으면 이후 이미지 로딩이 불가능하므로, 여기서 사용자에게 알릴 방법을 고려할 수 있습니다.
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
      connect(idle_timer_, &QTimer::timeout, this, &KeyboardPanel::showIdleImage);

      ros::NodeHandle nh;
      subscriber_key_ = nh.subscribe("/keyboard_input", 10, &KeyboardPanel::keyboardCallback, this);

      // 초기 이미지를 idle 상태로 설정
      showIdleImage();
    }

  private Q_SLOTS:
    void showIdleImage()
    {
      // pkg_path_가 비어있는 경우에 대한 방어 코드
      if (pkg_path_.empty()) {
        ROS_ERROR_THROTTLE(5.0, "KeyboardPanel: Package path for my_rviz_panel is not set. Cannot load idle image.");
        direction_label_->setText("Error: pkg_path not set\nCannot load idle.png");
        return;
      }
      std::string idle_image_path = pkg_path_ + "/resources/idle.png";
      QPixmap pixmap(QString::fromStdString(idle_image_path));

      if(pixmap.isNull()){
          ROS_WARN("KeyboardPanel: Failed to load idle image: %s", idle_image_path.c_str());
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
            // w, a, s, d 가 아닌 다른 입력이거나, "키를 뗐다"는 명시적 메시지가 있다면 여기서 처리
            // 현재 코드는 w,a,s,d 외에는 바로 idle로 처리하거나, 해당 이미지가 없다고 표시합니다.
            // 만약 키보드 퍼블리셔가 "stop" 같은 메시지를 보낸다면, 여기서 idle.png로 직접 연결할 수 있습니다.
            // image_file_suffix = "/resources/idle.png";
            is_known_key = false; // 인식되지 않은 키 또는 idle 상태로 바로 전환해야 하는 경우
        }

        if (pkg_path_.empty()) {
            ROS_ERROR_THROTTLE(5.0, "KeyboardPanel: Package path for my_rviz_panel is not set. Cannot load images.");
            direction_label_->setText("Error: pkg_path not set");
            if(is_known_key) { // 인식된 키였다면 타이머 시작 (오류지만 일단 시도)
                 idle_timer_->start(200); // 200 밀리초 후에 idle 상태로 (시간은 조절 가능)
            } else {
                showIdleImage(); // 인식되지 않은 키면 바로 idle
            }
            return;
        }

        if (!is_known_key) { // w, a, s, d 가 아닌 경우 (또는 명시적으로 idle로 가야하는 경우)
            showIdleImage();
            return; // 더 이상 이미지 로드 및 타이머 시작을 하지 않음
        }

        std::string full_path = pkg_path_ + image_file_suffix;
        QPixmap pixmap(QString::fromStdString(full_path));

        if(pixmap.isNull()){
            ROS_WARN_THROTTLE(2.0, "KeyboardPanel: Failed to load image: %s", full_path.c_str());
            direction_label_->setText(QString("Img not found:\n%1").arg(QString::fromStdString(image_file_suffix).split('/').last()));
        } else {
            direction_label_->setPixmap(pixmap);
        }

        // 인식된 키 (w,a,s,d) 이미지를 표시한 후, 일정 시간 뒤 idle로 돌아가도록 타이머 시작
        idle_timer_->start(200); // 200 밀리초 후에 idle 상태로 (시간은 조절 가능)
                                 // 사용자가 키를 계속 누르고 있다면, 키보드 입력 노드가 메시지를 반복적으로 발행해야
                                 // 이 콜백이 계속 호출되어 타이머가 재시작되고 idle로 넘어가지 않습니다.
                                 // 만약 키보드 노드가 키를 누를 때 한 번만 메시지를 보낸다면, 이 방식이 잘 동작합니다.
    }

  private:
    QLabel* direction_label_;
    ros::Subscriber subscriber_key_;
    std::string pkg_path_;
    QTimer* idle_timer_; // idle 상태로 돌아가기 위한 타이머
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::KeyboardPanel, rviz::Panel)
// KeyboardPanel.moc 파일은 빌드 시스템(catkin_make 또는 colcon build 시 AUTOMOC)에 의해 생성됩니다.
// 이 cpp 파일의 이름이 KeyboardPanel.cpp라고 가정하고, moc 파일을 인클루드합니다.
// 실제 moc 파일 이름은 클래스 이름과 헤더 파일 위치에 따라 달라질 수 있습니다.
// 일반적으로 소스 파일 (.cpp)의 끝에 #include "ClassName.moc" 형태로 추가합니다.
// 만약 헤더파일(KeyboardPanel.h)에 Q_OBJECT 매크로와 슬롯이 선언되어 있다면,
// 헤더파일에 대한 moc 파일이 생성되고, 그 moc 파일은 cpp 파일 내에서 인클루드 됩니다.
// 지금 코드는 모든 클래스 정의가 .cpp 파일 안에 있으므로, 이 파일 자체에 대한 moc이 필요합니다.
// 파일명을 KeyboardPanel.cpp로 저장했다면 다음 라인이 맞습니다.
#include "KeyboardPanel.moc"