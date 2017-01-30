//http://stackoverflow.com/questions/1120478/capturing-a-time-in-milliseconds
//http://www.cplusplus.com/doc/tutorial/files/

#include <ctime>
#include <fstream>
#include <Windows.h>
#include <math.h>

using namespace std;

int main() {
	SYSTEMTIME tv;
	GetSystemTime(&tv);

	char buffer[50];
	ofstream file;
	file.open("test.csv", ios::out);

	for(int i = 0; i < 1000; i++){
		sprintf(buffer, "%02d:%02d:%03d,%f\n", tv.wMinute, tv.wSecond, tv.wMilliseconds, cos(i*2*3.14159/1000));
		file << buffer;
	}

	file.close();

	return 0;
}
