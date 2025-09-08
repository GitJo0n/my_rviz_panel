#include "compass_widget.h"
#include <cmath> // M_PI 사용

CompassWidget::CompassWidget(QWidget *parent) : QWidget(parent), yaw_degrees_(0.0)
{
    // 위젯의 최소 크기를 설정하여 너무 작아지지 않게 합니다.
    setMinimumSize(150, 150);
}

void CompassWidget::setYaw(double yaw_radians)
{
    // ROS의 각도(radian)를 QPainter가 사용하는 각도(degree)로 변환
    yaw_degrees_ = yaw_radians * 180.0 / M_PI;
    update(); // 각도가 변경될 때마다 위젯을 다시 그리도록 요청
}

void CompassWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing); // 부드러운 그림을 위해 안티에일리어싱 설정

    int side = qMin(width(), height());
    painter.translate(width() / 2, height() / 2); // 좌표계를 위젯의 중심으로 이동
    painter.scale(side / 200.0, side / 200.0); // 위젯 크기에 맞춰 스케일링

    // 1. 나침반 배경 그리기 (원, 눈금 등)
    painter.setPen(Qt::black);
    painter.setBrush(Qt::lightGray);
    painter.drawEllipse(-95, -95, 190, 190);

    // 2. 글자(N, E, S, W) 그리기
    // painter의 좌표계를 회전시켜 글자를 배치합니다.
    painter.save(); // 현재 좌표계 상태 저장
    painter.rotate(-yaw_degrees_); // 뷰의 Yaw만큼 반대로 회전시켜 글자가 항상 올바른 방향을 가리키게 함
    
    painter.setFont(QFont("Arial", 20, QFont::Bold));
    painter.drawText(-12, -90, 24, 30, Qt::AlignCenter, "N");
    painter.drawText(-12, 70, 24, 30, Qt::AlignCenter, "S");
    painter.drawText(70, -15, 24, 30, Qt::AlignCenter, "E");
    painter.drawText(-94, -15, 24, 30, Qt::AlignCenter, "W");
    
    painter.restore(); // 저장했던 좌표계로 복원

    // 3. 나침반 바늘 그리기 (항상 위쪽을 가리키는 빨간 삼각형)
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    QPolygon needle;
    needle << QPoint(0, -60) << QPoint(15, 0) << QPoint(-15, 0);
    painter.drawPolygon(needle);
}