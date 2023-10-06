#include "mainwindow.h"

#include <QFileDialog>
#include "marine_acoustic_msgs/RawSonarImage.h"

#include <rosbag/view.h>
#include <QGraphicsPixmapItem>
#include <QDateTime>
#include <QDebug>
#include <QTimer>

namespace layer_tracker
{

MainWindow::MainWindow(int &argc, char ** argv, QWidget *parent) :QMainWindow(parent)
{
  ui_.setupUi(this);
  scene_ = new QGraphicsScene(this);
  ui_.graphicsView->setScene(scene_);
  ui_.graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);
  connect(ui_.horizontalScaleHorizontalSlider, &QSlider::valueChanged, this, &MainWindow::adjustScale);
  connect(ui_.verticalScaleVerticalSlider, &QSlider::valueChanged, this, &MainWindow::adjustScale);

  connect(ui_.minDbLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogramIfParametersChanged);
  connect(ui_.minDbLineEdit, &QLineEdit::textChanged, this, &MainWindow::setParametersChanged);

  connect(ui_.maxDbLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogramIfParametersChanged);
  connect(ui_.maxDbLineEdit, &QLineEdit::textChanged, this, &MainWindow::setParametersChanged);

  connect(ui_.depthLinesSpacingLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogramIfParametersChanged);
  connect(ui_.depthLinesSpacingLineEdit, &QLineEdit::textChanged, this, &MainWindow::setParametersChanged);

  connect(ui_.sliceAlphaLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogramIfParametersChanged);
  connect(ui_.sliceAlphaLineEdit, &QLineEdit::textChanged, this, &MainWindow::setParametersChanged);

  connect(ui_.sliceWidthLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogramIfParametersChanged);
  connect(ui_.sliceWidthLineEdit, &QLineEdit::textChanged, this, &MainWindow::setParametersChanged);


  connect(ui_.channelComboBox,  qOverload<int>(&QComboBox::activated), this, &MainWindow::updateEchogram);
  connect(ui_.reNoiseCheckBox, &QCheckBox::stateChanged, this, &MainWindow::updateEchogram);


  connect(ui_.sliceMinDbLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);
  connect(ui_.minSizeLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);
  connect(ui_.maxSizeLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);
  connect(ui_.minDepthLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);
  connect(ui_.maxDepthLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);
  connect(ui_.maxDurationLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateSlices);

  connect(ui_.rosTopicLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateROS);
  connect(ui_.rosTopicEnabledCheckBox, &QCheckBox::stateChanged, this, &MainWindow::updateROS);

  tracker_node_ = std::make_shared<TrackerNode>(argc, argv);
  ui_.channelComboBox->insertItem(0, "ROS");
}

MainWindow::~MainWindow()
{
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
  QMainWindow::resizeEvent(event);
  adjustScale();
}

void MainWindow::adjustScale()
{
  if(pixmap_item_)
  {
    auto target_box = ui_.graphicsView->frameRect();

    auto scalex = target_box.width()/double(pixmap_item_->pixmap().width());
    scalex *= ui_.horizontalScaleHorizontalSlider->value()*0.01;

    auto scaley = target_box.height()/double(pixmap_item_->pixmap().height());
    scaley *= ui_.verticalScaleVerticalSlider->value()*0.01;

    ui_.graphicsView->setTransform(QTransform::fromScale(scalex, scaley));
  }
}

void MainWindow::on_actionOpen_triggered()
{
  QString fname = QFileDialog::getOpenFileName(this,tr("Open"));
  if(!fname.isEmpty())
    openBags(std::vector<QString>(1,fname));
}

void MainWindow::openBags(const std::vector<QString> &fnames)
{
  for(auto fname: fnames)
  {
    try
    {
      rosbag::Bag bag;
      bag.open(fname.toStdString(), rosbag::BagMode::Read);
      qDebug() << fname;

      for(auto message: rosbag::View(bag))
      {
        if (message.getDataType() == "marine_acoustic_msgs/RawSonarImage")
        {
          auto topic = message.getTopic();
          if(ui_.channelComboBox->findText(topic.c_str()) == -1)
            ui_.channelComboBox->insertItem(ui_.channelComboBox->count(),topic.c_str());
          marine_acoustic_msgs::RawSonarImage::ConstPtr ping = message.instantiate<marine_acoustic_msgs::RawSonarImage>();

          if(!trackers_by_channel_[topic])
          {
            trackers_by_channel_[topic] = std::make_shared<Tracker>(bin_size_);
            trackers_by_channel_[topic]->setLayerMinimumDepth(minimum_depth_);
          }
          trackers_by_channel_[topic]->addPing(*ping);
        }
      }
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  updateEchogram();
}

void MainWindow::setParametersChanged()
{
  parameters_changed_ = true;
}

void MainWindow::updateEchogramIfParametersChanged()
{
  if(parameters_changed_)
  {
    parameters_changed_ = false;
    updateEchogram();
  }
}

void MainWindow::updateEchogram()
{
  float min_db = -100.0;
  float max_db = 10.0;
  float depth_lines_spacing = 100.0;
  bool ok;
  float value = ui_.minDbLineEdit->text().toFloat(&ok);
  if(ok)
    min_db = value;
  value = ui_.maxDbLineEdit->text().toFloat(&ok);
  if(ok)
    max_db = value;
  value = ui_.depthLinesSpacingLineEdit->text().toFloat();
  if(ok && value > 0)
    depth_lines_spacing = value;


  scene_->clear();
  pixmap_item_ = nullptr;

  uint32_t max_height = 0;

  std::string channel = ui_.channelComboBox->currentText().toStdString();
  std::shared_ptr<Tracker> tracker;
  if (channel == "ROS")
    tracker = tracker_node_->tracker();
  else
  {
    auto tracker_iterator = trackers_by_channel_.find(channel);
    if(tracker_iterator == trackers_by_channel_.end())
      return;

    tracker = tracker_iterator->second;
  }
  auto pings = tracker->pings();
  for(const auto &p: pings)
    max_height = std::max<uint32_t>(max_height, p->values().size());
  uint32_t ping_count = pings.size();
  
  qDebug() << channel.c_str() << ping_count << "x" << max_height;
  QImage echogram(ping_count, max_height, QImage::Format_Grayscale8);
  echogram.fill(Qt::black);
  uint32_t ping_number = 0;
  for(auto p: pings)
  {
    std::vector<float> ranges;
    for(int i = 0; i < p->values().size(); i++)
      ranges.push_back(i*p->binSize());
    
    const std::vector<float>* values = &p->values();
    if(ui_.reNoiseCheckBox->isChecked())
      values = &p->valuesReBackground();

    for(int i = 0; i < values->size(); i++)
    {
      auto range = i*p->binSize();
      echogram.scanLine(i)[ping_number] = std::max(0, std::min(254,int(255*((values->at(i)-min_db)/(max_db-min_db)))));
    }
    ping_number++;
  }
  pixmap_item_ = scene_->addPixmap(QPixmap::fromImage(echogram));

  float grid_spacing = depth_lines_spacing/tracker->binSize();
  QPen line_pen(Qt::red, 0, Qt::DotLine);

  for(float y = grid_spacing; y <= echogram.height(); y += grid_spacing)
    scene_->addLine(0.0, y, echogram.width(), y, line_pen);

  adjustScale();

  getSlices();
}

void MainWindow::updateSlices()
{
  bool need_update = false;

  bool ok;
  float value = ui_.sliceMinDbLineEdit->text().toFloat(&ok);
  if(ok && value != minimum_level_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMinimumLevel(value);
    minimum_level_ =  value;
    need_update = true;
  }

  value = ui_.minDepthLineEdit->text().toFloat(&ok);
  if(ok && value != minimum_depth_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMinimumDepth(value);
    minimum_depth_ =  value;
    need_update = true;
  }

  value = ui_.maxDepthLineEdit->text().toFloat(&ok);
  if(ok && value != maximum_depth_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMaximumDepth(value);
    maximum_depth_ =  value;
    need_update = true;
  }

  value = ui_.minSizeLineEdit->text().toFloat(&ok);
  if(ok && value != minimum_size_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMinimumSize(value);
    minimum_size_ =  value;
    need_update = true;
  }

  value = ui_.maxSizeLineEdit->text().toFloat(&ok);
  if(ok && value != maximum_size_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMaximumSize(value);
    maximum_size_ =  value;
    need_update = true;
  }

  value = ui_.maxDurationLineEdit->text().toFloat(&ok);
  if(ok && value != maximum_layer_duration_)
  {
    for(auto tracker: trackers_by_channel_)
      tracker.second->setLayerMaximumDuration(ros::Duration(value));
    maximum_layer_duration_ = value;
    need_update = true;

  }

  if(need_update)
    updateEchogram();
}

void MainWindow::getSlices()
{
  if(drawing_slices_)
  {
    restart_slice_drawing_ = true;
    return;
  }

  while(!drawing_slices_ || restart_slice_drawing_)
  {
    drawing_slices_ = true;
    restart_slice_drawing_ = false;

    std::string channel = ui_.channelComboBox->currentText().toStdString();
    std::shared_ptr<Tracker> tracker;
    if (channel == "ROS")
      tracker = tracker_node_->tracker();
    else
    {
      auto tracker_iterator = trackers_by_channel_.find(channel);
      if(tracker_iterator == trackers_by_channel_.end())
        return;

      tracker = tracker_iterator->second;
    }

    float alpha = 0.5;
    float width = 0.25;
    bool ok;
    float value = ui_.sliceAlphaLineEdit->text().toFloat(&ok);
    if(ok)
      alpha = value;
    value = ui_.sliceWidthLineEdit->text().toFloat(&ok);
    if(ok)
      width = value;

    std::map<ros::Time, int> ping_time_to_index;

    QPen slice_pen(QColor(0, 255, 0, 255*alpha), width);

    int ping_number = 0;
    auto bin_size = tracker->binSize();
    for(auto ping: tracker->pings())
    {
      ping_time_to_index[ping->timestamp()] = ping_number;
      auto slices = tracker->slices(ping->timestamp());
      for(const auto& slice: slices)
      {
        scene_->addLine(ping_number+.5, 0.5+(slice.minimumDepth()/bin_size), ping_number+.5, (slice.maximumDepth()/bin_size)-0.5, slice_pen);
      }
      qApp->processEvents();
      if(restart_slice_drawing_)
        break;
      ping_number++;
    }


    QPen layer_pen(QColor(0, 0, 255, 255*alpha), width*.5);
    QBrush layer_brush(QColor(0, 0, 255, 255*alpha*.25), Qt::SolidPattern);
    QPen layer_pen_low(QColor(0, 128, 128, 255*alpha), width*.5);
    QBrush layer_brush_low(QColor(0, 128, 128, 255*alpha*.25), Qt::SolidPattern);


    for(const auto& layer: tracker->getLayers())
    {
      auto depths = layer.depthRangesByTime();
      if(!depths.empty())
      {
        QPolygonF polygon;
        auto iterator = depths.begin();
        while(iterator != depths.end())
        {
          auto next = iterator;
          next++;
          if (next != depths.end())
          {
            float x1 = ping_time_to_index[iterator->first]+.5;
            float x2 = ping_time_to_index[next->first]+.5;
            polygon.push_back(QPointF(x1, iterator->second.second/bin_size));
            polygon.push_back(QPointF(x2, next->second.second/bin_size));
          }
          iterator = next;
        }

        auto riterator = depths.rbegin();
        while(riterator != depths.rend())
        {
          auto next = riterator;
          next++;
          if (next != depths.rend())
          {
            float x1 = ping_time_to_index[riterator->first]+.5;
            float x2 = ping_time_to_index[next->first]+.5;
            polygon.push_back(QPointF(x1, riterator->second.first/bin_size));
            polygon.push_back(QPointF(x2, next->second.first/bin_size));
          }
          riterator = next;
        }
        if(layer.averageDB() > minimum_level_)
          scene_->addPolygon(polygon, layer_pen, layer_brush);
        else
          scene_->addPolygon(polygon, layer_pen_low, layer_brush_low);
      }
    }
    
  }
  drawing_slices_ = false;
}

//todo, call ros spin once
void MainWindow::rosSpinOnce()
{
  if(ui_.rosTopicEnabledCheckBox->isEnabled())
  {
    ros::spinOnce();
    updateEchogram();
    QTimer::singleShot(500, this, &MainWindow::rosSpinOnce);
  }
  
}

void MainWindow::updateROS()
{
  if(ui_.rosTopicEnabledCheckBox->isChecked())
  {
    tracker_node_->setTopic(ui_.rosTopicLineEdit->text().toStdString());
    rosSpinOnce();
  }
  else
    tracker_node_->setTopic("");
}

} // namespace layer_tracker
