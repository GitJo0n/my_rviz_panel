#include "compass_widget.h"
#include <cmath> // M_PI 사용

CompassWidget::CompassWidget(QWidget *parent) 
    : QWidget(parent), yaw_degrees_(0.0), offset_degrees_(0.0) // <<-- offset_degrees_ 초기화
{
    setMinimumSize(150, 150);
}

void CompassWidget::setYaw(double yaw_radians)
{
    yaw_degrees_ = yaw_radians * 180.0 / M_PI;
    update();
}

// <<-- 각도 조절 함수 구현
void CompassWidget::adjustOffset(double degrees)
{
    offset_degrees_ += degrees;
    // 360도를 넘지 않도록 처리 (선택 사항)
    if (offset_degrees_ >= 360.0) offset_degrees_ -= 360.0;
    if (offset_degrees_ < 0.0) offset_degrees_ += 360.0;
    
    update(); // 보정 값이 변경되었으니 화면을 다시 그립니다.
}


void CompassWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int side = qMin(width(), height());
    painter.translate(width() / 2, height() / 2); // 좌표계를 위젯의 중심으로 이동
    painter.scale(side / 200.0, side / 200.0); // 위젯 크기에 맞춰 스케일링

    // 1. 나침반 배경(원)만 먼저 그립니다.
    painter.setPen(Qt::black);
    painter.setBrush(Qt::lightGray);
    painter.drawEllipse(-95, -95, 190, 190);

    // 2. 글자와 바늘을 함께 회전시키기 위해 painter 상태를 저장합니다.
    painter.save();

    // 장비의 방향(yaw)과 보정값(offset)을 기반으로 회전 각도를 계산합니다.
    double total_rotation = -yaw_degrees_ - offset_degrees_;
    painter.rotate(total_rotation);

    // --- 이 회전된 좌표계 안에서 글자와 바늘을 모두 그립니다 ---

    // 3. 글자(N, E, S, W) 그리기
    painter.setFont(QFont("Arial", 20, QFont::Bold));
    painter.drawText(-12, -95, 24, 30, Qt::AlignCenter, "N");
    painter.drawText(-12, 70, 24, 30, Qt::AlignCenter, "S");
    painter.drawText(70, -15, 24, 30, Qt::AlignCenter, "E");
    painter.drawText(-94, -15, 24, 30, Qt::AlignCenter, "W");

    // 4. 나침반 바늘 그리기 (빨간색 - 북쪽)
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    QPolygon north_needle;
    north_needle << QPoint(0, -60) << QPoint(15, 0) << QPoint(-15, 0); // 위쪽을 향하는 삼각형
    painter.drawPolygon(north_needle);

    // 5. 나침반 바늘 그리기 (파란색 - 남쪽)
    painter.setBrush(Qt::blue); // 파란색으로 변경
    QPolygon south_needle;
    south_needle << QPoint(0, 60) << QPoint(15, 0) << QPoint(-15, 0); // 아래쪽을 향하는 삼각형
    painter.drawPolygon(south_needle);

    // ---------------------------------------------------------

    // 6. painter 상태를 원래대로 복구합니다.
    painter.restore();
}

double CompassWidget::getCurrentDegrees() const
{
    double current_angle = yaw_degrees_ + offset_degrees_;
    
    // 각도를 0~360도 범위로 정규화합니다.
    double normalized_angle = fmod(current_angle, 360.0);
    if (normalized_angle < 0) {
        normalized_angle += 360.0;
    }
    return normalized_angle;
}