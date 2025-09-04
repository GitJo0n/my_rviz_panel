#ifndef COMPASS_WIDGET_H
#define COMPASS_WIDGET_H
#include <QWidget>
#include <QPainter>

class CompassWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CompassWidget(QWidget *parent = nullptr);

    // 외부에서 현재 각도를 설정할 함수
    void setYaw(double yaw_radians);

protected:
    // 이 함수에서 실제 그리기를 수행합니다.
    void paintEvent(QPaintEvent *event) override;

private:
    double yaw_degrees_; // painter는 각도(degree)를 사용합니다.
};
#endif // COMPASS_WIDGET_H