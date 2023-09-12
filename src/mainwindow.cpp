#include "mainwindow.h"

#include <QFileDialog>
#include "marine_acoustic_msgs/RawSonarImage.h"

#include <rosbag/view.h>
#include <QGraphicsPixmapItem>

#include <QDebug>

namespace layer_tracker
{

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent)
{
  ui_.setupUi(this);
  scene_ = new QGraphicsScene(this);
  ui_.graphicsView->setScene(scene_);
  connect(ui_.horizontalScaleHorizontalSlider, &QSlider::valueChanged, this, &MainWindow::adjustScale);
  connect(ui_.verticalScaleVerticalSlider, &QSlider::valueChanged, this, &MainWindow::adjustScale);
  connect(ui_.minDbLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogram);
  connect(ui_.maxDbLineEdit, &QLineEdit::editingFinished, this, &MainWindow::updateEchogram);
  connect(ui_.maxValuesCheckBox, &QCheckBox::stateChanged, this, &MainWindow::updateEchogram);
  connect(ui_.reNoiseCheckBox, &QCheckBox::stateChanged, this, &MainWindow::updateEchogram);
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

    pixmap_item_->setTransform(QTransform::fromScale(scalex, scaley));
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
          marine_acoustic_msgs::RawSonarImage::ConstPtr ping = message.instantiate<marine_acoustic_msgs::RawSonarImage>();
          pings_by_topic[topic][ping->header.stamp] = Ping(*ping, 0.1);
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

void MainWindow::updateEchogram()
{
  float min_db = -100.0;
  float max_db = 0.0;
  bool ok;
  float value = ui_.minDbLineEdit->text().toFloat(&ok);
  if(ok)
    min_db = value;
  value = ui_.maxDbLineEdit->text().toFloat(&ok);
  if(ok)
    max_db = value;

  if(display_min_db_ == min_db && display_max_db_ == max_db && use_max_ == ui_.maxValuesCheckBox->isChecked() && use_re_noise_ == ui_.reNoiseCheckBox->isChecked())
    return;
  scene_->clear();
  std::map<std::string, uint32_t> max_height_by_topic;
  std::map<std::string, uint32_t> ping_count_by_topic;
  for(auto& topic: pings_by_topic)
  {
    for(auto &p: topic.second)
      max_height_by_topic[topic.first] = std::max<uint32_t>(max_height_by_topic[topic.first], p.second.values().size());
    ping_count_by_topic[topic.first] = topic.second.size();
  }

  for(auto pc: ping_count_by_topic)
  {
    qDebug() << pc.first.c_str() << pc.second << "x" << max_height_by_topic[pc.first];
    QImage echogram(pc.second, max_height_by_topic[pc.first], QImage::Format_Grayscale8);
    echogram.fill(Qt::black);
    uint32_t ping_count = 0;
    for(auto p: pings_by_topic[pc.first])
    {
      std::vector<float> ranges;
      for(int i = 0; i < p.second.values().size(); i++)
        ranges.push_back(i*p.second.binSize());
      
      const std::vector<float>* values = &p.second.values();
      if(ui_.maxValuesCheckBox->isChecked())
      {
        if(ui_.reNoiseCheckBox->isChecked())
        {
          values = &p.second.maxValuesReNoise();
          //qDebug() << "max re noise";
        }
        else
        {
          values = &p.second.maxValues();
          //qDebug() << "max values";
        }
      }
      else
      {
        if(ui_.reNoiseCheckBox->isChecked())
        {
          values = &p.second.valuesReNoise();
          //qDebug() << "values re noise";
        }
        else
        {
          values = &p.second.values();
          //qDebug() << "values";
        }
      }

      for(int i = 0; i < values->size(); i++)
      {
        auto range = i*p.second.binSize();
        echogram.scanLine(i)[ping_count] = std::max(0, std::min(254,int(255*((*values)[i]-min_db)/(max_db-min_db))));
      }
      ping_count++;
    }
    pixmap_item_ = scene_->addPixmap(QPixmap::fromImage(echogram));
    adjustScale();
    break;
  }
  display_max_db_ = max_db;
  display_min_db_ = min_db;
  use_max_ = ui_.maxValuesCheckBox->isChecked();
  use_re_noise_ = ui_.reNoiseCheckBox->isChecked();
}

} // namespace layer_tracker
