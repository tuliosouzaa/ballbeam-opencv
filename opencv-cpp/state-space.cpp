#include "stdafx.h"

#define SERVO_OFFSET 105
#define KP 0.432
#define KI 0.0133
#define KD 0.463

using namespace cv;
using namespace std;

clock_t end_s;
clock_t begin_s;

HANDLE hDevice; 
DCB lpTest;
DWORD btsIO;

Mat image;
Point origin;
Rect selection;

bool backprojMode = false;
bool selectObject = false;
bool showHist = true;

float control_output;
float integrl,derivada;
float posx = 0;
float erro_anterior;
float yder, yfilt, yfilt_ant, Nbar;

double elapsed_secs;
double ttl;

int vmin = 10, vmax = 256, smin = 30;
int trackObject = 0;
int FLAG = 0;
int position;
int filtered_value=0;
int erro;
int MV = 240;
float y, y_ant;
float saida, saida_ant;

static void onMouse( int event, int x, int y, int, void* )
{
	if( selectObject )
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x,y);
		selection = Rect(x,y,0,0);
		selectObject = true;
		break;
	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		if( selection.width > 0 && selection.height > 0 )
			trackObject = -1;
		break;
	}

}

static void help()
{
	cout << "\nThis is a demo that shows mean-shift based tracking\n"
		"You select a color objects such as your face and it tracks it.\n"
		"This reads from video camera (0 by default, or the camera number the user enters\n"
		"Usage: \n"
		"   ./camshiftdemo [camera number]\n";

	cout << "\n\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tc - stop the tracking\n"
		"\tb - switch to/from backprojection view\n"
		"\th - show/hide object histogram\n"
		"\tp - pause video\n"
		"To initialize tracking, select the object with mouse\n";
}

void inicia_serial()
{
	hDevice = CreateFile(L"\\\\.\\COM8",GENERIC_READ | GENERIC_WRITE,FILE_SHARE_READ | FILE_SHARE_WRITE,NULL,OPEN_EXISTING,0,0);

	GetCommState(hDevice,&lpTest);
	lpTest.BaudRate = CBR_9600;
	lpTest.ByteSize = 8;
	lpTest.Parity = NOPARITY;
	lpTest.StopBits = ONESTOPBIT;
	SetCommState(hDevice,&lpTest);

	if (hDevice ==INVALID_HANDLE_VALUE) 
	{
		printf("DEU RUIM MONSTRAO \n");
		DWORD lastError = GetLastError();
		cout<<"ERROR HERE! = "<<lastError<<endl;
	}
	if (hDevice !=INVALID_HANDLE_VALUE) 
	{
		printf("DEU BOM MONSTRAO \n");
		DWORD lastError = GetLastError();
		cout<<"ERROR HERE! = "<<lastError<<endl;
	}
}

float filtro(int poss)
{
	if(FLAG < 5)
		filtered_value = filtered_value + poss;

	FLAG++;

	if(FLAG == 5)
	{
		int output1 = filtered_value/5;
		FLAG = 0;
		filtered_value = 0;		
		return (output1);
	}
	else
		return (0);


}

void serialWrite(int control)
{
	static char buffer[4];

	int control_send = 100 - control;	
	_itoa_s(control_send,buffer,10);
	
	if(control_send < 100) buffer[2]=',';
	buffer[3]=',';

	if (hDevice !=INVALID_HANDLE_VALUE)
	{
		printf("SERIAL SEND: %s \n \n",buffer);
		WriteFile(hDevice,buffer,strlen(buffer),&btsIO,NULL);
	}

}

const char* keys =
{
	"{1|  | 0 | camera number}"
};

int main( int argc, const char** argv )
{
	help();
	inicia_serial();

	//DEFINE VARIAVEIS NAO PERMANENTES INICIO
	VideoCapture cap;
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = {0,180};
	const float* phranges = hranges;
	CommandLineParser parser(argc, argv, keys);
	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;
	//FIM

	//INICIALIZA CAMERA INICIO
	cap.open(1);
	if( !cap.isOpened() )
	{
		help();
		cout << "***DEU RUIM DEMAIS COM A CAMERA BIXAO***\n";
		cout << "E O ERRO FOI... TAN TAN TAN TAN: \n";
		parser.printParams();
		return -1;
	}
	//INICIALIZA CAMERA FIM

	//INICIALIZACAO DA INTERFACE INICIO
	namedWindow( "Histogram", 0 );
	namedWindow( "CamShift Demo", 0 );
	namedWindow( "CFG", 0 );
	setMouseCallback( "CamShift Demo", onMouse, 0 );
	createTrackbar( "Vmin", "CamShift Demo", &vmin, 256, 0 );
	createTrackbar( "Vmax", "CamShift Demo", &vmax, 256, 0 );
	createTrackbar( "Smin", "CamShift Demo", &smin, 256, 0 );
	createTrackbar( "Posicao", "CFG", 0, 480, 0 );
	createTrackbar( "MV", "CFG", &MV, 480, 0 );
	//INICIALIZACAO DA INTERFACE FIM

	//INICIALIZA LOOP CAPTURE
	for(;;)
	{
		if( !paused )
		{
			cap >> frame;
			if( frame.empty() )
				break;
		}

		frame.copyTo(image);

		if( !paused )
		{
			cvtColor(image, hsv, COLOR_BGR2HSV);

			if( trackObject )
			{
				//TRANSFORMADA DE COR
				int _vmin = vmin, _vmax = vmax;
				inRange(hsv, Scalar(0, smin, MIN(_vmin,_vmax)),Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				int ch[] = {0, 0};
				hue.create(hsv.size(), hsv.depth());
				mixChannels(&hsv, 1, &hue, 1, ch, 1);

				if( trackObject < 0 )
				{
					//ROI DA SELECAO
					Mat roi(hue, selection), maskroi(mask, selection);
					//HISTOGRAM
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, CV_MINMAX);

					trackWindow = selection;
					trackObject = 1;

					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for( int i = 0; i < hsize; i++ )
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180./hsize), 255, 255);

					//COLOR FILTER
					cvtColor(buf, buf, CV_HSV2BGR);

					for( int i = 0; i < hsize; i++ )
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
						rectangle( histimg, Point(i*binW,histimg.rows),Point((i+1)*binW,histimg.rows - val),Scalar(buf.at<Vec3b>(i)), -1, 8 );
					}
				}

				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				//ROTATED RECT NA SELECAO
				RotatedRect trackBox = CamShift(backproj, trackWindow,TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));
				//TRACKWINDOW
				if( trackWindow.area() <= 1 )
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,trackWindow.x + r, trackWindow.y + r) &Rect(0, 0, cols, rows);				
				}

				if( backprojMode )
					cvtColor( backproj, image, COLOR_GRAY2BGR );
				//DESENHA ELIPSE NO TRACK SELECTION
				ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );
				//ENCONTRA O CENTRO DA TRACK SELECTION
				int posx = trackBox.center.x;
				posx = -0.756*posx + 491.45;
				setTrackbarPos("Posicao", "CFG", posx);
			
				position = filtro(posx);
				////////////////////////////////////////////////////////////
				////////////////////////////////////////////////////////////
				////////////////////////////////////////////////////////////
				//CONTROLADOR PID
				if (position != 0)
				{
					y = position/10;
					y = y/100; //divide por 100
			//		yfilt = 0.8939*y_ant + 0.1003*yfilt_ant;
				//	printf("yfilt %f \n",yfilt);
			//		yfilt_ant = yfilt;

					yder = (y - y_ant)/0.1; // deriva
					printf("yder %f \n",yder);
					y_ant = y;
					   
					int mvv = MV/10;
					float u = 70.4352*y + 44.7196*yder;
					control_output = mvv*0.7741 - u ;
					printf("SINAL DE CONTROLE %f \n",control_output);

					if (control_output > 30) 
						control_output = 30;
					else if (control_output < -30)
						control_output = -30;

					//SERIAL PRINT
					////////////////////////////////////////////////////////////
					////////////////////////////////////////////////////////////
					printf("SINAL DE CONTROLE %f \n",control_output);
					serialWrite(control_output);
					////////////////////////////////////////////////////////////
					////////////////////////////////////////////////////////////
					Sleep(100);
				}

			}
		}

		else if( trackObject < 0 )
			paused = false;

		if( selectObject && selection.width > 0 && selection.height > 0 )
		{
			Mat roi(image, selection);
			bitwise_not(roi, roi);
		}

		imshow( "CamShift Demo", image );

		imshow( "Histogram", histimg );

		char c = (char)waitKey(10);
		if( c == 27 )
			break;
		switch(c)
		{
		case 'b':
			backprojMode = !backprojMode;
			break;
		case 'c':
			trackObject = 0;
			histimg = Scalar::all(0);
			break;
		case 'h':
			showHist = !showHist;
			if( !showHist )
				destroyWindow( "Histogram" );
			else
				namedWindow( "Histogram", 1 );
			break;
		case 'p':
			paused = !paused;
			break;
		default:
			;
		}
	}

	return 0;
	CloseHandle(hDevice);
}