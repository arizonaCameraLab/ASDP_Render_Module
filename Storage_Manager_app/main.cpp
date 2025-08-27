/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include <QApplication>
#include "mainwindow.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();

    // Look for a non-Qt command-line argument (not starting with '-')
    QString serverArg;
    for (int i = 1; i < argc; ++i) {
      QString arg = argv[i];
      if (!arg.startsWith('-')) {
        serverArg = arg;
        break;
      }
    }

    if (!serverArg.isEmpty()) {
      std::cout << "Connecting to server: " << serverArg.toStdString() << std::endl;
      // Make an empty NIC name so that the client will be created without a discovery port.
      window.SelectNIC("");
      window.SelectServer(serverArg);
    }

    return app.exec();
}
