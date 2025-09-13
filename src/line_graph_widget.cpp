#include "mavlink_gui_tester/line_graph_widget.hpp"
#include <QtGui/QPaintEvent>
#include <QtGui/QFont>
#include <QtCore/QtMath>

namespace mavlink_gui_tester {

LineGraphWidget::LineGraphWidget(QWidget *parent)
    : QWidget(parent), max_points_(100), min_value_(0), max_value_(1), min_time_(0), max_time_(1) {
    setMinimumSize(400, 300);
    setStyleSheet("background-color: white; border: 1px solid gray;");
}

void LineGraphWidget::addDataPoint(const QString& series_name, double timestamp, double value) {
    if (!series_data_.contains(series_name)) {
        DataSeries new_series;
        new_series.visible = true;
        new_series.name = series_name;

        // Assign colors based on series name
        if (series_name.contains("Target")) new_series.color = Qt::blue;
        else if (series_name.contains("Current Angle")) new_series.color = Qt::red;
        else if (series_name.contains("Velocity")) new_series.color = Qt::green;
        else if (series_name.contains("Current")) new_series.color = Qt::magenta;
        else new_series.color = Qt::black;

        series_data_[series_name] = new_series;
    }

    series_data_[series_name].points.append(QPointF(timestamp, value));

    // Keep only max_points_ recent points
    if (series_data_[series_name].points.size() > max_points_) {
        series_data_[series_name].points.removeFirst();
    }

    updateBounds();
    update(); // Trigger repaint
}

void LineGraphWidget::clearSeries(const QString& series_name) {
    if (series_data_.contains(series_name)) {
        series_data_[series_name].points.clear();
        updateBounds();
        update();
    }
}

void LineGraphWidget::clearAll() {
    for (auto& series : series_data_) {
        series.points.clear();
    }
    updateBounds();
    update();
}

void LineGraphWidget::setSeriesVisible(const QString& series_name, bool visible) {
    if (series_data_.contains(series_name)) {
        series_data_[series_name].visible = visible;
        update();
    }
}

void LineGraphWidget::setMaxPoints(int max_points) {
    max_points_ = max_points;
    for (auto& series : series_data_) {
        while (series.points.size() > max_points_) {
            series.points.removeFirst();
        }
    }
    updateBounds();
    update();
}

void LineGraphWidget::updateBounds() {
    bool first_point = true;

    for (const auto& series : series_data_) {
        if (!series.visible || series.points.isEmpty()) continue;

        for (const auto& point : series.points) {
            if (first_point) {
                min_value_ = max_value_ = point.y();
                min_time_ = max_time_ = point.x();
                first_point = false;
            } else {
                min_value_ = qMin(min_value_, point.y());
                max_value_ = qMax(max_value_, point.y());
                min_time_ = qMin(min_time_, point.x());
                max_time_ = qMax(max_time_, point.x());
            }
        }
    }

    // Add some padding
    if (!first_point && max_value_ != min_value_) {
        double range = max_value_ - min_value_;
        min_value_ -= range * 0.1;
        max_value_ += range * 0.1;
    }
}

QPointF LineGraphWidget::mapToWidget(const QPointF& data_point, const QRect& graph_rect) {
    double x_ratio = (max_time_ != min_time_) ? (data_point.x() - min_time_) / (max_time_ - min_time_) : 0.5;
    double y_ratio = (max_value_ != min_value_) ? (data_point.y() - min_value_) / (max_value_ - min_value_) : 0.5;

    double x = graph_rect.left() + x_ratio * graph_rect.width();
    double y = graph_rect.bottom() - y_ratio * graph_rect.height(); // Invert Y axis

    return QPointF(x, y);
}

void LineGraphWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QRect widget_rect = rect();
    QRect graph_rect = widget_rect.adjusted(50, 30, -30, -50); // Margins for axes

    // Draw background
    painter.fillRect(widget_rect, Qt::white);
    painter.setPen(Qt::gray);
    painter.drawRect(graph_rect);

    // Draw grid lines
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));
    for (int i = 1; i < 5; ++i) {
        int y = graph_rect.top() + (graph_rect.height() * i / 5);
        painter.drawLine(graph_rect.left(), y, graph_rect.right(), y);

        int x = graph_rect.left() + (graph_rect.width() * i / 5);
        painter.drawLine(x, graph_rect.top(), x, graph_rect.bottom());
    }

    // Draw axes labels
    painter.setPen(Qt::black);
    painter.setFont(QFont("Arial", 8));

    // Y-axis labels
    for (int i = 0; i <= 5; ++i) {
        int y = graph_rect.bottom() - (graph_rect.height() * i / 5);
        double value = min_value_ + (max_value_ - min_value_) * i / 5;
        painter.drawText(QRect(5, y - 10, 40, 20), Qt::AlignRight | Qt::AlignVCenter,
                        QString::number(value, 'f', 1));
    }

    // Draw data series
    for (const auto& series : series_data_) {
        if (!series.visible || series.points.size() < 2) continue;

        painter.setPen(QPen(series.color, 2));

        for (int i = 1; i < series.points.size(); ++i) {
            QPointF p1 = mapToWidget(series.points[i-1], graph_rect);
            QPointF p2 = mapToWidget(series.points[i], graph_rect);
            painter.drawLine(p1, p2);
        }
    }

    // Draw legend
    int legend_y = 10;
    painter.setFont(QFont("Arial", 9));
    for (const auto& series : series_data_) {
        if (!series.visible) continue;

        painter.setPen(series.color);
        painter.fillRect(widget_rect.width() - 150, legend_y, 15, 10, series.color);
        painter.setPen(Qt::black);
        painter.drawText(widget_rect.width() - 130, legend_y + 10, series.name);
        legend_y += 20;
    }

    // Draw title
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.drawText(graph_rect, Qt::AlignTop | Qt::AlignHCenter, "Motor State Data");
}

} // namespace mavlink_gui_tester