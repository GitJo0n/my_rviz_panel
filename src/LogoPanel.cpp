#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
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
      label_ = new QLabel(this);
      QPixmap pixmap("/home/user/catkin_ws/src/my_rviz_panel/resources/logo.png");
      label_->setPixmap(pixmap);
      label_->setAlignment(Qt::AlignCenter);

      QVBoxLayout* layout = new QVBoxLayout;
      layout->addWidget(label_);
      setLayout(layout);

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
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::LogoPanel, rviz::Panel)
#include "LogoPanel.moc"
