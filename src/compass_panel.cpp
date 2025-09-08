#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <QVBoxLayout>
#include <QHBoxLayout>   // <<-- 버튼을 수평으로 배치하기 위해 추가
#include <QPushButton>  // <<-- 버튼 사용을 위해 추가
#include "compass_panel.h"

// Ogre 라이브러리의 헤더
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>


namespace my_compass_panel
{

CompassPanel::CompassPanel(QWidget* parent) : rviz::Panel(parent)
{
    // 1. 위젯들 생성
    compass_widget_ = new CompassWidget(this);
    QPushButton* rotate_left_button = new QPushButton("Rotate (-)", this);
    QPushButton* rotate_right_button = new QPushButton("Rotate (+)", this);

    // 2. 레이아웃 설정
    // 버튼들을 담을 수평 레이아웃
    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addWidget(rotate_left_button);
    button_layout->addWidget(rotate_right_button);

    // 전체 위젯을 담을 수직 레이아웃
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->addWidget(compass_widget_);
    main_layout->addLayout(button_layout); // 수평 레이아웃을 수직 레이아웃에 추가
    setLayout(main_layout);

    // 3. 시그널-슬롯 연결
    // 좌회전 버튼 클릭 시, compass_widget_의 각도를 -5도 조정
    connect(rotate_left_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(-5.0);
    });

    // 우회전 버튼 클릭 시, compass_widget_의 각도를 +5도 조정
    connect(rotate_right_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(5.0);
    });

    // 4. RViz 뷰 업데이트 타이머 설정 (기존과 동일)
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CompassPanel::onUpdate);
    timer_->start(100);
}

// onUpdate 함수는 수정할 필요가 없습니다.
// setYaw는 순수한 RViz 카메라의 yaw 값만 전달하고,
// 보정은 CompassWidget 내부에서 처리됩니다.
void CompassPanel::onUpdate()
{
    rviz::ViewController* view_controller = vis_manager_->getViewManager()->getCurrent();
    if (view_controller)
    {
        //const Ogre::Quaternion& orientation = view_controller->getCamera()->getOrientation();
        // 카메라의 방향(Orientation) 대신 위치(Position)를 가져옵니다.
        const Ogre::Vector3& position = view_controller->getCamera()->getPosition();

        // atan2(x, y)는 Y축(북쪽)을 기준으로 한 각도를 반환합니다.
        double yaw = atan2(position.x, position.y);
        compass_widget_->setYaw(yaw);

        // 현재 각도를 읽어와 라벨 업데이트 (이 부분은 그대로 둡니다)
        double current_degrees = compass_widget_->getCurrentDegrees();
        angle_label_->setText(degreesToCardinalString(current_degrees));
    }
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_compass_panel::CompassPanel, rviz::Panel)