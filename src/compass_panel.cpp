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
    angle_label_ = new QLabel("N 0", this);
    angle_label_->setAlignment(Qt::AlignCenter); // 텍스트 가운데 정렬
    angle_label_->setFont(QFont("Arial", 14, QFont::Bold));

    // 버튼 4개 생성
    QPushButton* coarse_left_button = new QPushButton("◀◀ (-5)", this);
    QPushButton* fine_left_button = new QPushButton("◀ (-1)", this);
    QPushButton* fine_right_button = new QPushButton("▶ (+1)", this);
    QPushButton* coarse_right_button = new QPushButton("▶▶ (+5)", this);

    // 2. 레이아웃 설정
    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addWidget(coarse_left_button);
    button_layout->addWidget(fine_left_button);
    button_layout->addWidget(fine_right_button);
    button_layout->addWidget(coarse_right_button);

    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->addWidget(compass_widget_);
    main_layout->addWidget(angle_label_); // 나침반과 버튼 사이에 라벨 추가
    main_layout->addLayout(button_layout);
    setLayout(main_layout);

    // 3. 시그널-슬롯 연결
    connect(coarse_left_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(-5.0);
    });
    connect(fine_left_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(-1.0);
    });
    connect(fine_right_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(1.0);
    });
    connect(coarse_right_button, &QPushButton::clicked, [this]() {
        compass_widget_->adjustOffset(5.0);
    });

    // 4. RViz 뷰 업데이트 타이머 설정
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CompassPanel::onUpdate);
    timer_->start(100);
}

void CompassPanel::onUpdate()
{
    rviz::ViewController* view_controller = vis_manager_->getViewManager()->getCurrent();
    if (view_controller)
    {

        const Ogre::Vector3& position = view_controller->getCamera()->getPosition();
        // Top-down 뷰 기준
        double yaw = atan2(position.x, position.y);
        compass_widget_->setYaw(yaw);
        
        // <<-- 현재 각도를 읽어와 라벨 업데이트
        double current_degrees = compass_widget_->getCurrentDegrees();
        angle_label_->setText(degreesToCardinalString(current_degrees));
    }
}

// <<-- 각도를 "NE 38도" 형식의 문자열로 변환하는 함수
QString CompassPanel::degreesToCardinalString(double degrees)
{
    // 16방위 표기
    static const QString directions[] = {
        "N", "NNE", "NE", "ENE", 
        "E", "ESE", "SE", "SSE", 
        "S", "SSW", "SW", "WSW", 
        "W", "WNW", "NW", "NNW"
    };

    // 360/16 = 22.5도 간격
    int index = static_cast<int>(round(degrees / 22.5));
    // 359.9... 가 16으로 계산되는 경우를 대비해 0~15 사이 값으로 만듦
    index = index % 16; 
    
    QString cardinal = directions[index];
    
    // QString::asprintf() 나 .arg()를 사용하여 문자열 조합
    return QString("%1 %2도").arg(cardinal).arg(static_cast<int>(round(degrees)));
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_compass_panel::CompassPanel, rviz::Panel)