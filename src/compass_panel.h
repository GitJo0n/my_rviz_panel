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
    CompassWidget* compass_widget_;
    QTimer* timer_;
};
} // end namespace my_compass_panel

#endif // COMPASS_PANEL_H