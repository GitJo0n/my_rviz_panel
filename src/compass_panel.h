#ifndef COMPASS_PANEL_H
#define COMPASS_PANEL_H

#include <rviz/panel.h>
#include "compass_widget.h" // 방금 만든 위젯 헤더
#include <QTimer>

namespace my_compass_panel
{
class CompassPanel : public rviz::Panel
{
    Q_OBJECT
public:
    CompassPanel(QWidget* parent = 0);

protected Q_SLOTS:
    void onUpdate(); // 주기적으로 호출될 함수

private:
    QLabel* angle_label_; // 현재 각도를 표시할 라벨
    CompassWidget* compass_widget_;
    QTimer* timer_;
    QString degreesToCardinalString(double degrees); // 각도를 방위각 문자열로 변환하는 함수
};
} // end namespace my_compass_panel

#endif // COMPASS_PANEL_H