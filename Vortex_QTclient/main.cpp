#include "mainwindow.h"

#include <QApplication>
#include <QCommandLineParser>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    
    QCommandLineParser parser;
    parser.addOption(QCommandLineOption("test", "测试模式：启动时连接TCP，可手动输入指令"));
    parser.process(a);
    
    MainWindow w(parser.isSet("test"));
    w.show();
    return a.exec();
}
