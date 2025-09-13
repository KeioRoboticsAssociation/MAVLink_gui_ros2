#ifndef LINE_GRAPH_WIDGET_HPP
#define LINE_GRAPH_WIDGET_HPP

#include <QtWidgets/QWidget>
#include <QtGui/QPainter>
#include <QtGui/QPen>
#include <QtCore/QPointF>
#include <QtCore/QMap>
#include <QtCore/QVector>
#include <QtGui/QColor>

namespace mavlink_gui_tester {

class LineGraphWidget : public QWidget {

public:
    explicit LineGraphWidget(QWidget *parent = nullptr);

    void addDataPoint(const QString& series_name, double timestamp, double value);
    void clearSeries(const QString& series_name);
    void clearAll();
    void setSeriesVisible(const QString& series_name, bool visible);
    void setMaxPoints(int max_points);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    struct DataSeries {
        QVector<QPointF> points;
        QColor color;
        bool visible;
        QString name;
    };

    QMap<QString, DataSeries> series_data_;
    int max_points_;
    double min_value_, max_value_;
    double min_time_, max_time_;

    void updateBounds();
    QPointF mapToWidget(const QPointF& data_point, const QRect& graph_rect);
};

} // namespace mavlink_gui_tester

#endif // LINE_GRAPH_WIDGET_HPP