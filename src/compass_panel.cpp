#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <QVBoxLayout>
#include "compass_panel.h"

// Ogre 라이브러리의 헤더 (Quaternion, Radian 등 사용)
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>


namespace my_compass_panel
{

CompassPanel::CompassPanel(QWidget* parent) : rviz::Panel(parent)
{
    compass_widget_ = new CompassWidget(this);
    
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(compass_widget_);
    setLayout(layout);
    
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CompassPanel::onUpdate);
    timer_->start(100); // 0.1초 (100ms) 마다 onUpdate 호출
}

void CompassPanel::onUpdate()
{
    // getDisplayContext()를 통해 RViz의 핵심 기능에 접근
    rviz::ViewController* view_controller = vis_manager_->getViewManager()->getCurrent();
    if (view_controller)
    {
        // 현재 뷰 컨트롤러에서 카메라의 방향(Quaternion)을 가져옴
        const Ogre::Quaternion& orientation = view_controller->getCamera()->getOrientation();
        
        // Quaternion에서 Yaw 각도(Radian)를 추출
        double yaw = orientation.getYaw().valueRadians();
        
        // 나침반 위젯에 새로운 각도 설정
        compass_widget_->setYaw(yaw);
    }
}
} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_compass_panel::CompassPanel, rviz::Panel)