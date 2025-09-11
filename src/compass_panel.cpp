#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/view_controller.h>
#include <rviz/properties/float_property.h> // ViewController의 속성 제어
#include <QVBoxLayout>
#include <QHBoxLayout>   // 버튼을 수평으로 배치하기 위해 
#include <QPushButton>  // 버튼 사용
#include <QSlider>      // 슬라이더 사용
#include "compass_panel.h"

// Ogre 라이브러리의 헤더
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>


namespace my_compass_panel
{

CompassPanel::CompassPanel(QWidget* parent) : rviz::Panel(parent)
{
    // 나침반 관련 위젯
    compass_widget_ = new CompassWidget(this);
    angle_label_ = new QLabel("N 0°", this);
    angle_label_->setAlignment(Qt::AlignCenter);
    angle_label_->setFont(QFont("Arial", 14, QFont::Bold));

    QPushButton* coarse_left_button = new QPushButton("◀◀ -5°", this);
    QPushButton* fine_left_button = new QPushButton("◀ -1°", this);
    QPushButton* fine_right_button = new QPushButton("▶ +1°", this);
    QPushButton* coarse_right_button = new QPushButton("▶▶ +5°", this);

    // 줌 관련 위젯
    zoom_slider_ = new QSlider(Qt::Vertical, this); // 수직 슬라이더
    zoom_slider_->setRange(0, 100); // 줌 레벨을 0~100 단계
    zoom_slider_->setValue(50);     // 초기값
    zoom_slider_->setInvertedAppearance(false); // 위로 올리면 값이 커지도록

    QPushButton* zoom_in_button = new QPushButton("+", this);
    QPushButton* zoom_out_button = new QPushButton("-", this);

    // main_layout
    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->addWidget(coarse_left_button);
    button_layout->addWidget(fine_left_button);
    button_layout->addWidget(fine_right_button);
    button_layout->addWidget(coarse_right_button);
    
    QVBoxLayout* compass_layout = new QVBoxLayout();
    compass_layout->addWidget(compass_widget_);
    compass_layout->addWidget(angle_label_);
    compass_layout->addLayout(button_layout);

    // 줌 부분 수직 레이아웃
    QVBoxLayout* zoom_layout = new QVBoxLayout();

    QHBoxLayout* zoom_in_wrapper = new QHBoxLayout();
    zoom_in_wrapper->addStretch(); // 왼쪽에 빈 공간 추가
    zoom_in_wrapper->addWidget(zoom_in_button);
    zoom_in_wrapper->addStretch(); // 오른쪽에 빈 공간 추가
    zoom_layout->addLayout(zoom_in_wrapper);

    // zoom_slider_를 가운데 정렬하기 위한 '래퍼(wrapper)' 레이아웃
    QHBoxLayout* slider_wrapper = new QHBoxLayout();
    slider_wrapper->addStretch();
    slider_wrapper->addWidget(zoom_slider_);
    slider_wrapper->addStretch();
    zoom_layout->addLayout(slider_wrapper);

    // zoom_out_button을 가운데 정렬하기 위한 '래퍼(wrapper)' 레이아웃
    QHBoxLayout* zoom_out_wrapper = new QHBoxLayout();
    zoom_out_wrapper->addStretch();
    zoom_out_wrapper->addWidget(zoom_out_button);
    zoom_out_wrapper->addStretch();
    zoom_layout->addLayout(zoom_out_wrapper);

    QWidget* zoom_container = new QWidget();
    zoom_container->setStyleSheet(
        "background-color: white;"
        "border-radius: 10px;" // 모서리를 둥글게 만듭니다.
    );
    zoom_container->setLayout(zoom_layout);
    // (C) 전체를 묶는 최종 수평 레이아웃 (stretch factor 제거)
    QHBoxLayout* main_layout = new QHBoxLayout(this);
    main_layout->addLayout(compass_layout); // 왼쪽 나침반
    main_layout->addWidget(zoom_container); // 오른쪽 줌
    setLayout(main_layout);

    // 나침반 버튼 연결
    connect(coarse_left_button, &QPushButton::clicked, [this](){ compass_widget_->adjustOffset(-5.0); });
    connect(fine_left_button, &QPushButton::clicked, [this](){ compass_widget_->adjustOffset(-1.0); });
    connect(fine_right_button, &QPushButton::clicked, [this](){ compass_widget_->adjustOffset(1.0); });
    connect(coarse_right_button, &QPushButton::clicked, [this](){ compass_widget_->adjustOffset(5.0); });

    // 줌 위젯 연결
    connect(zoom_slider_, &QSlider::valueChanged, this, &CompassPanel::onZoomChanged);
    connect(zoom_in_button, &QPushButton::clicked, [this](){ zoom_slider_->setValue(zoom_slider_->value() + 5); });
    connect(zoom_out_button, &QPushButton::clicked, [this](){ zoom_slider_->setValue(zoom_slider_->value() - 5); });

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &CompassPanel::onUpdate);
    timer_->start(100);
}

void CompassPanel::onZoomChanged(int value)
{
    rviz::ViewController* view_controller = vis_manager_->getViewManager()->getCurrent();
    if (view_controller)
    {
        rviz::Property* untyped_prop = view_controller->subProp("Distance");

        // FloatProperty 포인터로 형 변환
        rviz::FloatProperty* distance_prop = qobject_cast<rviz::FloatProperty*>(untyped_prop);

        if (distance_prop)
        {
            // 슬라이더 값(0~100)을 실제 거리 값(예: 1~50)으로 변환
            float min_dist = 1.0;
            float max_dist = 50.0;
            float new_dist = min_dist + (max_dist - min_dist) * (100 - value) / 100.0f;
            
            distance_prop->setFloat(new_dist);
        }
    }
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
    return QString("%1 %2˚").arg(cardinal).arg(static_cast<int>(round(degrees)));
}

} // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(my_compass_panel::CompassPanel, rviz::Panel)