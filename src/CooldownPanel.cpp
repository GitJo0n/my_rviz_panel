#include <rviz/panel.h>
#include <QTimer>
#include <QLabel>
#include <QHBoxLayout>
#include <vector>
#include <pluginlib/class_list_macros.h>

namespace my_rviz_panel
{
    class CooldownPanel : public rviz::Panel
    {
        Q_OBJECT
    public:
        CooldownPanel(QWidget* parent = 0) : rviz::Panel(parent), cooldown_step_(0)
        {
            QHBoxLayout* layout = new QHBoxLayout;

            for (int i = 0; i < 5; ++i) {
                QLabel* label = new QLabel;
                label->setFixedSize(20, 20);
                label->setStyleSheet("background-color: gray;");
                layout->addWidget(label);
                boxes_.push_back(label);
            }

            setLayout(layout);

            timer_ = new QTimer(this);
            connect(timer_, &QTimer::timeout, this, &CooldownPanel::updateCooldownDisplay);
        }

        // 외부에서 호출 (예: 마커 발행 직후)
        void startCooldown()
        {
            cooldown_step_ = 0;
            for (auto& box : boxes_) {
                box->setStyleSheet("background-color: gray;");
            }
            timer_->start(1000); // 1초마다 1개씩 초록으로
        }

    private Q_SLOTS:
        void updateCooldownDisplay()
        {
            if (cooldown_step_ < 5) {
                boxes_[cooldown_step_]->setStyleSheet("background-color: green;");
                ++cooldown_step_;
            } else {
                timer_->stop();
            }
        }

    private:
        QTimer* timer_;
        std::vector<QLabel*> boxes_;
        int cooldown_step_;
    };
}


// 플러그인 등록
PLUGINLIB_EXPORT_CLASS(my_rviz_panel::CooldownPanel, rviz::Panel)
#include "CooldownPanel.moc"
