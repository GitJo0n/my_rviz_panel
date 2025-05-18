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
  class KeyboardPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    KeyboardPanel(QWidget* parent = 0)
      : rviz::Panel(parent)
    {
      pkg_path_ = ros::package::getPath("my_rviz_panel");

      direction_label_ = new QLabel(this);
      QPixmap default_pixmap(QString::fromStdString(pkg_path_ + "/resources/idle.png"));
      direction_label_->setPixmap(default_pixmap);
      direction_label_->setAlignment(Qt::AlignCenter);
      direction_label_->setScaledContents(true);
      direction_label_->setFixedSize(612, 328); // 크기 조정 필요시 주석 해제
      direction_label_->setFixedSize(612, 328);

      QVBoxLayout* layout = new QVBoxLayout;
      layout->addWidget(direction_label_);
      setLayout(layout);

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

        std::string full_path = pkg_path_ + image_file;
        QPixmap pixmap(QString::fromStdString(full_path));
        direction_label_->setPixmap(pixmap);
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
