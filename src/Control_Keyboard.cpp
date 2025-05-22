#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QString>
#include <QFrame>
#include <QTimer>
#include <QKeyEvent>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_list_macros.h>

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
        ROS_ERROR("Control_Keyboard: Could not find package path.");
      }

      // 포커스를 받도록 설정
      setFocusPolicy(Qt::StrongFocus);
      setFocus();

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

      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

      direction_label_ = new QLabel(content_frame);
      direction_label_->setAlignment(Qt::AlignCenter);
      direction_label_->setScaledContents(true);
      //direction_label_->setFixedSize(400, 200);
      direction_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

      QVBoxLayout* frame_layout = new QVBoxLayout(content_frame);
      frame_layout->addWidget(direction_label_);

      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      QVBoxLayout* panel_layout = new QVBoxLayout(this);
      panel_layout->addWidget(content_frame);
      setLayout(panel_layout);

      idle_timer_ = new QTimer(this);
      idle_timer_->setSingleShot(true);
      connect(idle_timer_, &QTimer::timeout, this, &Control_Keyboard::showIdleImage);

      showIdleImage();
    }

  protected:
    void keyPressEvent(QKeyEvent* event) override
    {
      QString key_text = event->text().toLower();

      if (key_text == "w") {
        cmd_vel.linear.x = 0.7;
      } else if (key_text == "s") {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
      } else if (key_text == "a") {
        cmd_vel.angular.z = -0.6;
      } else if (key_text == "d") {
        cmd_vel.angular.z = 0.6;
      } else {
        return;
      }
      cmd_vel_pub_.publish(cmd_vel);
      updateDirectionImageFromTwist(cmd_vel);
      // idle_timer_->stop();

      // QString key_text = event->text().toLower();
      // std::string image_file_suffix;
      // bool is_known_key = true;

      // if (key_text == "w") image_file_suffix = "/resources/forward.png";
      // else if (key_text == "s") image_file_suffix = "/resources/backward.png";
      // else if (key_text == "a") image_file_suffix = "/resources/left.png";
      // else if (key_text == "d") image_file_suffix = "/resources/right.png";
      // else is_known_key = false;

      // if (pkg_path_.empty()) {
      //   ROS_ERROR_THROTTLE(5.0, "Package path not set");
      //   direction_label_->setText("Error: pkg_path not set");
      //   return;
      // }

      // if (!is_known_key) {
      //   showIdleImage();
      //   return;
      // }

      // std::string full_path = pkg_path_ + image_file_suffix;
      // QPixmap pixmap(QString::fromStdString(full_path));

      // if (pixmap.isNull()) {
      //   ROS_WARN_THROTTLE(2.0, "Failed to load image: %s", full_path.c_str());
      //   direction_label_->setText(QString("Image not found:\n%1").arg(QString::fromStdString(image_file_suffix).split('/').last()));
      // } else {
      //   direction_label_->setPixmap(pixmap);
      // }

      // idle_timer_->start(200);  // 일정 시간 뒤 idle 이미지로 복귀
    }
    void updateDirectionImageFromTwist(const geometry_msgs::Twist& cmd_vel)
    {
      std::string image_path;

      if (cmd_vel.linear.x > 0) {
        if (cmd_vel.angular.z > 0) {
          image_path = "/resources/forward_right.png";
        } else if (cmd_vel.angular.z < 0) {
          image_path = "/resources/forward_left.png";
        } else {
          image_path = "/resources/forward.png";
        } 
      } else {
        if (cmd_vel.angular.z > 0) {
          image_path = "/resources/right.png";
        } else if (cmd_vel.angular.z < 0) {
          image_path = "/resources/left.png";
        } else {
          showIdleImage();
          return;
        }
      }

      std::string full_path = pkg_path_ + image_path;
      QPixmap pixmap(QString::fromStdString(full_path));
      if (!pixmap.isNull()) {
        direction_label_->setPixmap(pixmap.scaled(direction_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
      } else {
        direction_label_->setText("Image not found");
      }
    }

    ros::Publisher cmd_vel_pub_;
    ros::NodeHandle nh_;

  private Q_SLOTS:
    void showIdleImage()
    {
      if (pkg_path_.empty()) {
        direction_label_->setText("Error: pkg_path not set");
        return;
      }

      std::string idle_image_path = pkg_path_ + "/resources/idle.png";
      QPixmap pixmap(QString::fromStdString(idle_image_path));

      if (pixmap.isNull()) {
        ROS_WARN("Idle image not found: %s", idle_image_path.c_str());
        direction_label_->setText("idle.png not found");
      } else {
        direction_label_->setPixmap(
          pixmap.scaled(direction_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation)
        );
      }
    }

  private:
    QLabel* direction_label_;
    std::string pkg_path_;
    QTimer* idle_timer_;
    geometry_msgs::Twist cmd_vel;
  };

}  // namespace my_rviz_panel

PLUGINLIB_EXPORT_CLASS(my_rviz_panel::Control_Keyboard, rviz::Panel)
#include "Control_Keyboard.moc"
