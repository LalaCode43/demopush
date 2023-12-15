#include "mainwindow.h"

#include <QApplication>
#include <iostream>
#include <librealsense2/rs.hpp>
int main(int argc, char *argv[])try
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
