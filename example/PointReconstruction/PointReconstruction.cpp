#include <QApplication>
#include "widget.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    PointWindow window;
    window.show();                               

    return app.exec();
}