#include <QApplication>

#include "ui/OffLineProgrammingSimulationMainWindow.h"

int main(int argc, char *argv[]) {
    qDebug()<<"Hello"<<endl;
    QApplication a(argc, argv);
    OffLineProgrammingSimulationMainWindow w;
    w.show();

    return a.exec();
}
