#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QString>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>

namespace my_rviz_panel
{
  class KOREATECH : public rviz::Panel
  {
    Q_OBJECT
  public:
    KOREATECH(QWidget* parent = 0)
      : rviz::Panel(parent)
    {
      // 패키지 경로 구하기
      pkg_path_ = ros::package::getPath("my_rviz_panel");

      // 로고 라벨 설정
      logo_label_ = new QLabel(this);
      QPixmap logo_pixmap(QString::fromStdString(pkg_path_ + "/resources/logo.png"));
      logo_label_->setPixmap(logo_pixmap);
      logo_label_->setAlignment(Qt::AlignCenter);
      logo_label_->setScaledContents(true);
      logo_label_->setFixedSize(200, 200);

      // 방향 라벨 설정 (기본 idle 이미지)
      direction_label_ = new QLabel(this);
      QPixmap default_pixmap(QString::fromStdString(pkg_path_ + "/resources/idle.png"));
      direction_label_->setPixmap(default_pixmap);
      direction_label_->setAlignment(Qt::AlignCenter);
      direction_label_->setScaledContents(true);
      direction_label_->setFixedSize(100, 100);

      // 레이아웃
      QVBoxLayout* layout = new QVBoxLayout;
      layout->addWidget(logo_label_);
      layout->addWidget(direction_label_);
      setLayout(layout);

      // ROS subscriber
      ros::NodeHandle nh;
      subscriber_text_ = nh.subscribe("/status_text", 10, &KOREATECH::updateText, this);
      subscriber_key_ = nh.subscribe("/keyboard_input", 10, &KOREATECH::updateDirectionImage, this);
    }

  private Q_SLOTS:
    void updateText(const std_msgs::String::ConstPtr& msg)
    {
      // 텍스트는 무시하고 이미지만 남기기로 한 경우 생략 가능
    }

    void updateDirectionImage(const std_msgs::String::ConstPtr& msg)
    {
      QString image_file;

      if (msg->data == "w") {
        image_file = "/resources/forward.png";
      } else if (msg->data == "s") {
        image_file = "/resources/backward.png";
      } else if (msg->data == "a") {
        image_file = "/resources/left.png";
      } else if (msg->data == "d") {
        image_file = "/resources/right.png";
      } else {
        image_file = "/resources/idle.png";
      }

      QPixmap pixmap(QString::fromStdString(pkg_path_ + image_file));
      direction_label_->setPixmap(pixmap);
    }

  private:
    QLabel* logo_label_;
    QLabel* direction_label_;
    ros::Subscriber subscriber_text_;
    ros::Subscriber subscriber_key_;
    std::string pkg_path_;
  };

}  // namespace my_rviz_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::KOREATECH, rviz::Panel)
#include "my_rviz_panel.moc"
