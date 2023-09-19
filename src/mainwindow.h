#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_layertrackermainwindow.h"
#include "ping.h"

namespace layer_tracker
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  void resizeEvent(QResizeEvent* event) override;


public slots:
  void openBags(const std::vector<QString> &fnames);
  void adjustScale();
  void updateEchogram();
  void getSlices();

private slots:
  void on_actionOpen_triggered();

private:
  Ui::LayerTrackerMainWindow ui_;
  QGraphicsScene* scene_ = nullptr;
  QGraphicsPixmapItem* pixmap_item_ = nullptr;

  using Pings = std::map<ros::Time, Ping>;
  std::map<std::string, Pings> pings_by_topic;

  float display_min_db_ = 0.0;
  float display_max_db_ = 0.0;
  bool use_max_ = false;
  bool use_re_noise_ = false;
  int display_integration_count_ = 0;
  std::string display_channel_;
};

} // namespace layer_tracker

#endif // MAINWINDOW_H
