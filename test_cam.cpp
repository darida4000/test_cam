
  /////////////////////////////////////////////////////////////////////

  // RICICLO PROGETTO DIFFERENZIATA
  //
  //

  /////////////////////////////////////////////////////////////////////



  /////////////////////////////////////////////////////////////////////

  // INCLUDES

  /////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <utility>



#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>





// COSTANTI
#define BUF_SIZE 128
#define IMG_OBJECT 8 // immagini nel database
#define IMG_SCENE 3 // numero fotogrammi
#define ORB_PRECISION 5000 // precisione orb

using namespace cv;
using namespace std;

// PROTOTIPI

bool niceHomography(Mat* H);
double homographyRating(Mat* H);
void readImages();
int findMaxIndex();
void captureImages(int cam);
bool calcolaEmd();
void  salva_dati_thingspeack(string codice_tessera, string mat, float peso);
std::vector<std::string> explode(std::string const & s, char delim);
void readConfig();
int checkAll();
string imageDetection();
string colorQuickWin();

  /////////////////////////////////////////////////////////////////////

  // DICHIARAZIONI

  /////////////////////////////////////////////////////////////////////

// OpenCV
int maxImages=IMG_OBJECT; // togliere
int objectIndex=0;
Mat img_object;
Mat img_scene[IMG_SCENE];
VideoCapture cap;
bool found=false;
Mat img_object_data[IMG_OBJECT];
Mat img_matches[IMG_OBJECT];
double determinanti[IMG_OBJECT][IMG_SCENE];
Ptr<Feature2D> f2d;
Ptr<FeatureDetector> detector;
Ptr<DescriptorExtractor> extractor;
Mat descriptors_object[IMG_OBJECT], descriptors_scene[IMG_SCENE];
std::vector<KeyPoint> keypoints_object[IMG_OBJECT], keypoints_scene[IMG_SCENE];
String nomiRifiuti[IMG_OBJECT];
int colindex=0;
double maxFound=0;
Mat bg; Mat carta[2];
BFMatcher matcher(NORM_HAMMING);
  
  // RFID
  int i=0;
  int k=0;
  int cport_nr_arduino=25;
  int bdrate_arduino=57600;
  int cport_nr=24;
  int bdrate=19200;
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  char str_send[BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  char codice_tessera_hex[6];


int main()
{
	cout << "Accensione" << endl << flush;
	char risultato[100];
	
  /////////////////////////////////////////////////////////////////////

  // CONTROLLI

  /////////////////////////////////////////////////////////////////////

   if(checkAll()==-1) return 0;
 	f2d= ORB::create();
    
    detector = ORB::create(ORB_PRECISION);
	readImages();

  /////////////////////////////////////////////////////////////////////

  // INIZIALIZZAZIONI

  /////////////////////////////////////////////////////////////////////
  cap.open(0);
      for(int i=0;i<10;i++) cap >> img_scene[0];
        
      for(int i=0;i<10;i++) cap >> img_scene[1];

      for(int i=0;i<10;i++) cap >> img_scene[2];
    
	cout << "immagini catturate";
	
	for (int i=0;i< 3;i++)
	{

		detector->detectAndCompute(img_scene[i], noArray(), keypoints_scene[i], descriptors_scene[i]);

	}
	
	

    bool trovato=false;
 
	string r = colorQuickWin();
	if(r != "")
	{
		sprintf(risultato,"%s",r.c_str());
		trovato = true;
	}

		
			// vedo se è carta
	if(trovato == false) 
	{
		if((calcolaEmd()==true) )
		{
			sprintf(risultato,"C - Carta");
			trovato = true;
		}
		
	}
	
	return 0;
		// metodo orb
	if(trovato == false) // se non è escuso, passo al video
	{
		r = imageDetection();
		if(r != "")
		{
			sprintf(risultato,"%s",r.c_str());
			trovato = true;
		}
	}
	
	

   


   cap.release();

  return(0);
}



  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO

  /////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO

  /////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO

  /////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Legge le immagini dal disco

  /////////////////////////////////////////////////////////////////////


void readImages()
{

    
    // per ORB
    
  img_object_data[0] = imread("img/mk.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[0] = "Milka";
  img_object_data[1] = imread("img/cr2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[1] = "Croccantelle";
  img_object_data[2] = imread("img/cr1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[2] = "Croccantelle";
  img_object_data[3] = imread("img/sc2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[3] = "Schiacciatelle";
  img_object_data[4] = imread("img/fz1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[4] = "Fonzie";
  img_object_data[5] = imread("img/ps1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[5] = "Pizzottelle";
  img_object_data[6] = imread("img/sc3.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[6] = "Schiacciatelle";
  img_object_data[7] = imread("img/fz2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[7] = "Fonzie";

  for(int i =0;i< maxImages; i++) for(int ii =0;ii< 3; ii++) determinanti[i][ii] = 0;

	for (int i=0;i< maxImages;i++)
	{
	
		detector->detectAndCompute(img_object_data[i], noArray(), keypoints_object[i], descriptors_object[i]);
		/*if(descriptors_object[i].type()!=CV_32F) {
          descriptors_object[i].convertTo(descriptors_object[i], CV_32F);
        }*/


	}
}

  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Acquisisce immagini da webcam

  /////////////////////////////////////////////////////////////////////

void captureImages(int cam)
{

	cout << "Inizio cattura img";

    //try
   // {
      for(int i=0;i<10;i++) cap >> img_scene[0];
        
      for(int i=0;i<10;i++) cap >> img_scene[1];
      return;
      for(int i=0;i<10;i++) cap >> img_scene[2];
    
    //}
    //      catch(cv::Exception& e)
    //{


    //}

	cout << "immagini catturate";
	
	for (int i=0;i< 3;i++)
	{

		detector->detectAndCompute(img_scene[i], noArray(), keypoints_scene[i], descriptors_scene[i]);

        /*if(descriptors_scene[i].type()!=CV_32F) {
          descriptors_scene[i].convertTo(descriptors_scene[i], CV_32F);
        }*/

	}
	
}

  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Cotrollo omografia per orb

  /////////////////////////////////////////////////////////////////////
  
bool niceHomography(Mat* H)
{
	const double det = H->at<double>(0,0)* H->at<double>(1,1)-H->at<double>(1,0)* H->at<double>(0,1);
	if (det<0) return false;
	const double N1 = sqrt(H->at<double>(0,0)* H->at<double>(0,0)+H->at<double>(1,0)* H->at<double>(0,1));
	if (N1>4 || N1 < 0.1) return false;
	const double N2 = sqrt(H->at<double>(0,1)* H->at<double>(0,1)+H->at<double>(1,1)* H->at<double>(1,1));
	if (N2>4 || N2 < 0.1) return false;
	const double N3 = sqrt(H->at<double>(2,0)* H->at<double>(2,0)+H->at<double>(2,1)* H->at<double>(2,1));
	if (N3>0.002) return false;

	return true;


}

double homographyRating(Mat* H)
{

    const double det = H->at<double>(0,0)* H->at<double>(1,1)-H->at<double>(1,0)* H->at<double>(0,1);
    return det;

}

  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Trova il massimo valore nella matrice dei determinanti per ORB

  /////////////////////////////////////////////////////////////////////
  
int findMaxIndex()
{

    /*cout << "determinanti:" << endl;
    cout << "--------------" << endl;
*/
    double max=0;
    int indice = -1;
    int k;
    for( k=0;k< maxImages;k++)
    {

      for(int h=0;h<3;h++)
      {
  //      cout << " " << determinanti[k][h];
        if (determinanti[k][h] > 1) continue;
        if(determinanti[k][h] > max)
        {

          max = determinanti[k][h];
          indice = k;
          colindex=h;
        }
      }
      cout << endl;
    }
    maxFound=max;
    return indice;

}

  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // calcola EMD

  /////////////////////////////////////////////////////////////////////
  
bool calcolaEmd()
{

    
    Mat src_base, hsv_base;
    Mat src_test1, hsv_test1;
    Mat src_test2, hsv_test2;
    Mat hsv_half_down;


    int imgCarta=2;
   
    Mat scene[3];
    Mat hsv_carta[imgCarta];
    Mat hsv_scene[3];
    Mat mask_carta[imgCarta];
    Mat mask_scene[3];

    // carico immagine sfondo
    // per EMD
    bg=imread("img/sfondo.jpg",CV_LOAD_IMAGE_GRAYSCALE);

    carta[0] = imread( "img/cartabiancaok.jpg",1);
    carta[1] = imread( "img/cartamarroneok.jpg",1); // sostituire con carta bianca
	
	Mat maschera,mm;
    Mat backupFrame;
    
    string s="";
    string t="";
	
	// cerco di eliminare lo sfondo dalle immagini catturate
	/*
	 *     bg=imread("sfondo.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	 * 
			src_base = imread( "mano.jpg" , CV_LOAD_IMAGE_GRAYSCALE);
			absdiff(bg,src_base,src_test1);
			threshold(src_test1,src_test1,40,255,THRESH_BINARY);
			src_base = imread( "mano.jpg");
			src_base.copyTo(src_test2, src_test1);


		imshow("sottratta",src_test2);

	 * */
	for(int i=0;i<IMG_SCENE ;i++)
	{
		backupFrame = img_scene[i].clone();
		cvtColor( img_scene[i], img_scene[i], cv::COLOR_BGR2GRAY  );
		absdiff(bg,img_scene[i],maschera);
		threshold(maschera,maschera,40,255,THRESH_BINARY);
		backupFrame.copyTo(mm, maschera);
		s="img" + i;
		t="imgaaaaaaa" + i;
		imshow(s,mm);
		imshow(t,backupFrame);

	}
	
	waitKey(0);
	
	
	
    
    /*
     * 
     * 
     * 
     * 
   
		bg=imread("sfondo.jpg", CV_LOAD_IMAGE_GRAYSCALE);
		src_base = imread( "mano.jpg" , CV_LOAD_IMAGE_GRAYSCALE);
		absdiff(bg,src_base,src_test1);
		threshold(src_test1,src_test1,40,255,THRESH_BINARY);
		src_base = imread( "mano.jpg");
		src_base.copyTo(src_test2, src_test1);
		imshow("sottratta",src_test2);
		* 
		* 
   */
    
    for(int i=0;i<imgCarta;i++)
      {
        carta[i] = carta[i] - bg; // correggere la sottrazione

      /// Converto in HSV
        cvtColor( carta[i], hsv_carta[i], COLOR_BGR2HSV );



    inRange(hsv_carta[i], Scalar(0, 15, 50), Scalar(180, 255, 255), mask_carta[i]);

      }
  

    for(int i=0;i<3;i++)
      {
        scene[i] = img_scene[i].clone();
        scene[i] = scene[i] - bg;
        cvtColor( scene[i], hsv_scene[i], COLOR_BGR2HSV );
        inRange(hsv_scene[i], Scalar(0, 15, 50), Scalar(180, 255, 255), mask_scene[i]);
      }


   // setup
    int h_bins = 20; int s_bins = 20; int v_bins =50;
    int histSize[] = { h_bins, s_bins };

    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    float v_ranges[] = { 100, 256 };

    const float* ranges[] = { h_ranges,  s_ranges };


    int channels[] = { 0,1 };

    MatND hist_base[imgCarta];
    MatND hist_test[3];

    for (int i=0;i<imgCarta;i++)
    {
      calcHist( &hsv_carta[i], 1, channels, mask_carta[i], hist_base[i], 2, histSize, ranges, true, false );
      normalize( hist_base[i], hist_base[i], 0, 1, NORM_MINMAX, -1, Mat() );
    }
    for (int i=0;i<3;i++)
    {
      calcHist( &hsv_scene[i], 1, channels, mask_scene[i], hist_test[i], 2, histSize, ranges, true, false );
      normalize( hist_test[i], hist_test[i], 0, 1, NORM_MINMAX, -1, Mat() );
    }


  // calcolo EMD
  for(int i=0;i<imgCarta;i++)   /// per ogni img carta
  {
	  
  for (int j=0;j<3;j++) // per ogni fotogramma
  {
	
  vector<cv::Mat> sig(3);
  MatND hist[2];

  hist[0] = hist_base[i].clone();
  hist[1] = hist_test[j].clone();

  for(int i = 0;i<2;i++)
  {
    vector <cv::Vec3f> sigv;
    normalize( hist[i], hist[i], 1, 0, NORM_L1 );

    for(int h=0;h<h_bins;h++)
    {
      for(int s=0;s< s_bins;s++)
       {

        float bin_val=hist[i].at<float>(h,s);
        if(bin_val!=0) sigv.push_back(cv::Vec3f(bin_val,(float)h,(float)s));

      }
      sig[i] =cv::Mat(sigv).clone().reshape(1);


    }
    if(i>0){

        float emdResult = EMD(sig[0],sig[i],cv::DIST_L1);
        cout << "EMD:" << emdResult << endl;
        if (emdResult<4000) return true;
     }
    }
   }
 }


return false;
}

  /////////////////////////////////////////////////////////////////////

  // CHIAMATA CURL PER THINGSPEACK

  /////////////////////////////////////////////////////////////////////
  
void salva_dati_thingspeack(string codice_tessera, string mat, float peso)
{


}
  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Esplode una stringa

  /////////////////////////////////////////////////////////////////////
  
std::vector<std::string> explode(std::string const & s, char delim)
{
    std::vector<std::string> result;
    std::istringstream iss(s);

    for (std::string token; std::getline(iss, token, delim); )
    {
        result.push_back(token);
    }

    return result;
}

  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Legge file configurazione

  /////////////////////////////////////////////////////////////////////
  
void readConfig()
{
	ifstream fp;
	char line[100];
	std::vector<std::string> v ;
	String s;
	
	fp.open("config.riciclo");
	fp >> line;
	
	s=line;
	v = explode(s, '=');
    cport_nr_arduino = atof(v[1].c_str());  
	fp >> line;
	
	s=line;
	v = explode(line, '=');
    cport_nr = atof(v[1].c_str()); 	
    
    fp.close();
    
   int p=24;
   /* 
   while(1)
   {
	cout << "Cerco arduino in porta " << p << endl << flush;

	int n = RS232_PollComport(p, str_recv, (int)BUF_SIZE);
    if(n > 0){
        str_recv[n] = 0;
		cout << "Arduino è sulla "<< p  << endl << flush;
		
        break; 
    }
    p++;
    if(p==26) p = 24;
	usleep(1000000);  // 1 secondo di pausa
   } // fine ciclo lettura su tessera

    if(p==24)
    {
		cport_nr_arduino=24;
		cport_nr=25;
	}
	else
	{
		cport_nr_arduino=25;
		cport_nr=24;
		
	}

	 
   */
	
}


  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Controlli iniziali

  /////////////////////////////////////////////////////////////////////
  
  
int checkAll()
{


  try
	{
		if(!cap.open(0))
		{
			cout << "Errore apertura cam " << 0;
        }
    }
    catch(cv::Exception& e)
    {
		sleep(3000);
		if(!cap.open(0))
		{
			cout << "Errore apertura cam " << 0;
			return -1;
        }
	}

}
  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Applica quickwins basati sul colore prevalente  dell'oggetto

  /////////////////////////////////////////////////////////////////////
string colorQuickWin()
{
    

    string ris="";
    double pixel; 
    double perc;
    Mat test2_mask,hsv_test2;
    
    // controllo sul terzo fotogramma
    cvtColor( img_scene[2], hsv_test2, COLOR_BGR2HSV );

	// da ripetere per ogni prodotto
	
    inRange(hsv_test2, Scalar(25, 15, 50), Scalar(33, 255, 255), test2_mask); // valore per fonzie con percentuali intorno al 15
    pixel = countNonZero(test2_mask); //  307200 pixel totali per immagini 640 x 480
    perc = pixel*100/307200;
    if (perc > 15) 
    {
		ris = "P - Fonzies"; 
		return ris;
	}
	cout << perc << endl << flush;

	// fine da ripetere per ogni prodotto

	//inRange(hsv_test2, Scalar(10, 100, 100), Scalar(20, 255, 255), test2_mask); // valore per croccantelle con percentuali intorno al 15

    return ris;
 
}
  /////////////////////////////////////////////////////////////////////

  // FUNZIONI APPOGGIO
  // Applica metodo ORB

  /////////////////////////////////////////////////////////////////////
  
string imageDetection()
{
	string ris="";
 // confronto ogni immagine del database con i tre fotogrammi acquisiti

    for (int imgDb=0; imgDb<maxImages;imgDb++)
    {
		for (int imgIndex=1;imgIndex<3;imgIndex++)
		{
		// dichiaro e alloco alcuni oggetti che mi serviranno
			std::vector< DMatch > matches;
			double max_dist = 0; double min_dist = 100;
   
            matcher.match( descriptors_object[imgDb], descriptors_scene[imgIndex], matches );

			for( int i = 0; i < descriptors_object[imgDb].rows; i++ )
			{ double dist = matches[i].distance;
				if( dist < min_dist ) min_dist = dist;
				if( dist > max_dist ) max_dist = dist;
			}

			std::vector< DMatch > good_matches;

			for( int i = 0; i < descriptors_object[imgDb].rows; i++ )
			{ if( matches[i].distance <= 3*min_dist )
            { good_matches.push_back( matches[i]); }
			}

			std::vector<Point2f> obj;
			std::vector<Point2f> scene;

			for( int i = 0; i < good_matches.size(); i++ )
			{
          
				obj.push_back( keypoints_object[imgDb][ good_matches[i].queryIdx ].pt );
				scene.push_back( keypoints_scene[imgIndex][ good_matches[i].trainIdx ].pt );
			}
        
			if(good_matches.size()>4)
			{
				Mat H = findHomography( obj, scene, CV_RANSAC );
	            if((niceHomography(&H) == true) || (min_dist<100)) 
				{	
					determinanti[imgDb][imgIndex] = homographyRating(&H);
				}
			}
        
		} 
	}

	int index = findMaxIndex();
	if((index == -1) || (maxFound < 0.1))
	{
		ris="";
	}
	else
	{

		ris = "P - " + nomiRifiuti[index];
    }
  return ris;
}
