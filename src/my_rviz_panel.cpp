#include <rviz/panel.h>
#include <QLabel>
#include <QVBoxLayout>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QPixmap>

namespace my_rviz_panel
{
  class KOREATECH : public rviz::Panel
  {
    Q_OBJECT
  public:
    KOREATECH(QWidget* parent = 0)
      : rviz::Panel(parent)
    {
      // Create the label
      label_ = new QLabel(this);
      QPixmap pixmap("/home/user/catkin_ws/src/my_rviz_panel/resources/logo.png");
      label_->setPixmap(pixmap);
      label_->setAlignment(Qt::AlignCenter);      
      // Create a layout and add the label to it
      QVBoxLayout* layout = new QVBoxLayout;
      layout->addWidget(label_);
      setLayout(layout);

      // Create ROS subscriber
      ros::NodeHandle nh;
      subscriber_ = nh.subscribe("/status_text", 10, &KOREATECH::updateText, this);
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

}  // namespace my_rviz_panel

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::KOREATECH, rviz::Panel) // panel name
#include "my_rviz_panel.moc"
