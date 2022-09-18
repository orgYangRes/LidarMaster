#include "LidarMaster.h"
#include <QtWidgets/QApplication>
#include "vtkAutoInit.h"


VTK_MODULE_INIT(vtkRenderingOpenGL2);

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    LidarMaster w;
    w.show();
    return a.exec();
}
