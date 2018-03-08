
#include "IMUReceiver.h"
//#include "tracker/GLWidget.h"
#include <QObject>

/*
IMUReceiver::IMUReceiver(){
	std::cout << "Calling the constructor(qobject) from the void one" << std::endl;
	IMUReceiver(this);

}
*/

IMUReceiver::IMUReceiver(QObject* qobj){

	udpSocket = new QUdpSocket(qobj);
	udpSocket->bind(45454, QUdpSocket::ShareAddress);

	std::cout << "\n\nIMUReceiver constructed\n\n" << std::endl;
	connect(udpSocket, SIGNAL(readyRead()), qobj, SLOT(setValue(int)));
}


//void IMUReceiver::setValue(int value){
//	std::cout << "In the processPendingDatagrams function" << endl;
	
//}


/*
IMUReceiver::IMUReceiver(QObject* qobj){
	udpSocket = new QUdpSocket(qobj);
	udpSocket->bind(45454, QUdpSocket::ShareAddress);
	

	std::cout << "\n\nIMUReceiver constructed\n\n" << std::endl;

	//qobj->connect(udpSocket, SIGNAL(readyRead()), qobj, SLOT(processPendingDatagrams()));

	
}
*/