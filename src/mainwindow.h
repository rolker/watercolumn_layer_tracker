#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ui_layertrackermainwindow.h"
#include "tracker_node.h"

namespace layer_tracker
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(int &argc, char ** argv, QWidget *parent = 0);
  ~MainWindow();

  void resizeEvent(QResizeEvent* event) override;

public slots:
  void openBags(const std::vector<QString> &fnames);
  void adjustScale();
  void updateEchogram();
  void getSlices();
  void updateSlices();
  void setParametersChanged();
  void updateEchogramIfParametersChanged();
  void updateROS();
  void rosSpinOnce();

private slots:
  void on_actionOpen_triggered();

private:
  Ui::LayerTrackerMainWindow ui_;
  QGraphicsScene* scene_ = nullptr;
  QGraphicsPixmapItem* pixmap_item_ = nullptr;

  std::map<std::string, std::shared_ptr<Tracker> > trackers_by_channel_;

  std::shared_ptr<TrackerNode> tracker_node_;

  bool drawing_slices_ = false;
  bool restart_slice_drawing_ = false;

  bool parameters_changed_ = false;

  float bin_size_ = 0.25;
  float minimum_level_ = 0.0;
  float minimum_depth_ = 5.0;
  float maximum_depth_ = 500.0;
  float minimum_size_ = 1.0;
  float maximum_size_ = 25.0;
  float maximum_layer_duration_ = 30.0;
};

} // namespace layer_tracker

#endif // MAINWINDOW_H
