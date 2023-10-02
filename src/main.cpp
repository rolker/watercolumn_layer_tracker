#include "mainwindow.h"
#include <QApplication>


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  layer_tracker::MainWindow w(argc, argv);
  w.show();

  std::vector<QString> fnames;
  for(int i = 1; i < argc; i++)
    fnames.push_back(argv[i]);
  if(!fnames.empty())
    w.openBags(fnames);

  return a.exec();
}
