#pragma once

#include <QGraphicsView>
#include <QApplication>
#include <QtWidgets/QMainWindow>
#include <utility>

#include "base/PathStatistics.hpp"
#include "base/Trajectory.h"

struct LegendEntry
{
    std::string label;
    QPen pen;

    explicit LegendEntry(const std::string &label = "Unknown", const QPen &pen = QPen(Qt::black))
            : label(label), pen(pen)
    {}
};


class Environment;
class GNode;

class VisualizationView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit VisualizationView(QGraphicsScene *scene, QWidget *parent = nullptr);

    ~VisualizationView() override;

public slots:
    void showContextMenu(const QPoint &pos);
    void saveSvg();
    void savePng();

protected:
    void wheelEvent(QWheelEvent *event) override;

private:
    float _numScheduledScalings;
    QGraphicsScene *_scene;
};

class QtVisualizer
{
public:
    static void initialize();
    static int exec();
    static void show()
    {
        QtVisualizer::drawLegend();
        _window->showMaximized();
    }

    static void visualize(Environment *environment, int run, bool renderDistances = false);

    static void drawNode(const GNode &node,
                         QColor color = Qt::red,
                         double radius = 0.3,
                         bool drawArrow = true);
    static void drawNode(const Tpoint &point,
                         QColor color = Qt::red,
                         double radius = 0.3);
    static void drawNode(double x, double y,
                         QColor color = Qt::red,
                         double radius = 0.3);

    static void drawTrajectory(std::vector<GNode> nodes, const QColor &color = Qt::white,
                               float penWidth = 1.f, Qt::PenStyle penStyle = Qt::PenStyle::SolidLine);
    static void drawTrajectory(const GNode &a, const GNode &b, const QColor &color = Qt::white,
                               float penWidth = 1.f, Qt::PenStyle penStyle = Qt::PenStyle::SolidLine);
    static void drawPath(std::vector<GNode> nodes, const QColor &color = Qt::white,
                         float penWidth = 1.f, Qt::PenStyle penStyle = Qt::PenStyle::SolidLine);
    static void drawPath(std::vector<Tpoint> nodes, QColor color = Qt::white,
                         float penWidth = 1.f, Qt::PenStyle penStyle = Qt::PenStyle::SolidLine);
    static void drawPath(std::vector<Tpoint> nodes, QPen pen);
    static void drawNodes(std::vector<GNode> nodes, bool drawArrows = false,
                          QColor color = Qt::red, double radius = 0.3);
    static void drawNodes(std::vector<Tpoint> nodes,
                          QColor color = Qt::red, double radius = 0.3);

    static void drawLabel(const std::string &text, double x, double y,
                          QColor color = Qt::black, float size = 1.f);

    static void drawStats(const PathStatistics &stats);

    static void addLegendEntry(LegendEntry entry);

    static void drawLegend();

    static void showStartGoal(bool show = true);

    static void savePng(const QString &fileName);
    static void saveSvg(const QString &fileName);

    static void saveScene();
    static void restoreScene();

    virtual ~QtVisualizer();

private:
    QtVisualizer() = default;

    static VisualizationView *_view;
    static QApplication *_app;
    static QMainWindow *_window;
    static QGraphicsScene *_scene;
    static int _statsTextTop;

    static std::vector<LegendEntry> _legend;

    static bool _showStartGoal;

    static std::list<QGraphicsItem*> _storedItems;
};
