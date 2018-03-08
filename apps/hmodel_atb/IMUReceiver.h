#pragma once
#include <QtNetwork>
#include <QObject>
//#include <QGLWidget>
#include <stdio.h>
#include <iostream>
//?
//#include <WS2tcpip.h>
//#include <IPHlpApi.h>

//#pragma comment(lib, "Ws2_32.lib");

#define DEFAULT_PORT "27015"

//class QUdpSocket;

class IMUReceiver : public QObject {
	Q_OBJECT

	QUdpSocket *udpSocket = nullptr;
	QObject* widget;

public:
	IMUReceiver(QObject *parent = 0) : QObject(parent){

		udpSocket = new QUdpSocket(this);
		udpSocket->bind(QHostAddress("192.168.1.5"), 45454);
		//udpSocket->bind(45454, QUdpSocket::ShareAddress);

		connect(udpSocket, SIGNAL(readyRead()), this, SLOT(rxDataEvent()),Qt::QueuedConnection);

	};

	void setWidget(QObject* qobj){
		std::cout << "set IMURec to widget?" << std::endl;
		//udpSocket->bind(QHostAddress("192.168.1.5"), 45454);
		//udpSocket->bind(45454, QUdpSocket::ShareAddress);
		//udpSocket->bind(45454, QUdpSocket::ShareAddress);
		//udpSocket->

		//connect(udpSocket, SIGNAL(readyRead()), qobj, SLOT(rxDataEvent()), Qt::QueuedConnection);
	}

	void HelloUDP(){
		std::cout << "\nGonna send some UDP data \n" << std::endl;
		QByteArray Data;
		Data.append("Hello from UDP");
		QHostAddress addr("192.168.1.5");
		udpSocket->writeDatagram(Data, addr, 58451);
		udpSocket->writeDatagram(Data, addr, 45454);
	};

	/*
	IMUReceiver(){

		udpSocket = new QUdpSocket(this);
		udpSocket->bind(QHostAddress::LocalHost, 45454);
		//udpSocket->bind(45454, QUdpSocket::ShareAddress);

		std::cout << "\n\nIMUReceiver constructed\n\n" << std::endl;
		connect(udpSocket, SIGNAL(readyRead()), this, SLOT(setValue()));
	};
	*/

void check(){
	std::cout << "Pending data: " << udpSocket->pendingDatagramSize() << std::endl;
	if (udpSocket->hasPendingDatagrams()){
		rxDataEvent();
	}
}
	
signals:

public slots:
void setValue(){
	std::cout << "what is the slto?" << std::endl;
};


void rxDataEvent(){
	std::cout << "\n\n READY READ \n\n" << std::endl;
	QByteArray buffer;
	buffer.resize(udpSocket->pendingDatagramSize());

	QHostAddress sender;
	quint16 senderPort;

	udpSocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

	std::cout << "Message from: " << sender.toString().toUtf8().constData() << std::endl;
	//std::cout << "Message port: " << senderPort.toString().toUtf8().constData() << std::endl;
	std::cout << "Message: " << buffer.constData() << std::endl;

}

};