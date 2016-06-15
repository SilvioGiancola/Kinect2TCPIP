#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QTcpServer>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QDir>
#include <QThread>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void newTCPIPConnection();
    void newMessageReceived();
    void clientStateChanged(QAbstractSocket::SocketState);


private:
    Ui::MainWindow *ui;

    QTcpServer *server;
    QTcpSocket *socket;
    int _port;


};
#endif // MAINWINDOW_H
