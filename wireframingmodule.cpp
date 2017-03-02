
//
//  wireframingmodule.cpp
//  Opencv Motion Library
//
//  Created by Krishnan Raghavan on 5/1/14.
//  Copyright (c) 2014 Krishnan Raghavan. All rights reserved.
//

//#include "wireframingmodule.h"
#include "function definitions.h"

int main ()
{	
	L_List L_object ;
	
	I_List I_object ;
	
	P_List P_object ;

	M_List M_object ;

    cout << "Hello  This is the start of an awesome adventure "<< endl ; 

	string s = "1211.jpg" ;

    cout << "1"<<endl  ; 
    
    ImageMat *Iinp = new ImageMat;

    ImageMat *Iinp1 = new ImageMat;

    //img1=imread("1.JPG");\

    Size size(640,480);
    //the dst image size,e.g.100x100

	Iinp = ImageAcquisition(2,s,s,I_object);
	
	resize(Iinp->img,Iinp->img,size);//resize image

	string s1 = "1212.jpg" ;

	Iinp1 = ImageAcquisition(2,s1,s1,I_object);

	resize(Iinp1->img,Iinp1->img,size);

    //   for (int i =  0 ; i < 10 ; i ++){
	// storeimage(I_object,*Iinp, "Hello");
	// cout << "The size of the key is:    "<<I_object.keyouterImages.size()<<endl;
    //    cout << "The size of the key is:    "<<I_object.keytimeImages.size()<<endl; 
    //    } 
	// storeimage(I_object,*Iinp, "hi");
	// cout << "The size of the key is:    "<<I_object.keyouterImages.size()<<endl;
 //    cout << "The size of the key is:    "<<I_object.keytimeImages.size()<<endl; 

 //    cout << "Now removing "<< endl ;
 //    time_t t = time(0);
 //   	eraseimages(I_object,t, "Hello");

	// cout << "The size of the key is:    "<<I_object.keyouterImages.size()<<endl;
 //    cout << "The size of the key is:    "<<I_object.keytimeImages.size()<<endl; 
    //VideoCapture cap(0) ;

    //Mat img ; 

    //cap >> img ; 

    //Iinp->img = img.clone();

 //    Iinp=  PrelimEdgeDetection(Iinp , "EdgeImage1",I_object);

	// Iinp = VerticesDetection(Iinp, "VERTICESMAGE1",P_object , L_object , I_object);

 //    Iinp1=  PrelimEdgeDetection(Iinp1 , "EdgeImage2",I_object);

	// Iinp1 = VerticesDetection(Iinp1, "VERTICESMAGE2",P_object , L_object , I_object);


cvtColor(Iinp->img,Iinp->img,CV_RGB2GRAY);
cvtColor(Iinp1->img,Iinp1->img,CV_RGB2GRAY);
// 	cv::Matx34d P,P1;
cv::Mat K , Kinv ;
cv::Mat cam_matrix,distortion_coeff;
std::vector<CloudPoint> pointcloud;
std::vector<cv::KeyPoint> correspImg1Pt;
cv::FileStorage fs;
fs.open("out_camera_data.xml",cv::FileStorage::READ);
fs["Camera_Matrix"]>>cam_matrix;
fs["Distortion_Coefficients"]>>distortion_coeff;


K = cam_matrix;

invert(K, Kinv); //get inverse of camera matrix

cout << Kinv<<endl ;



int a ;
cin >> a ; 
cout<< "cam matrixes"<< K<< endl;

vector<KeyPoint> pt1,pt2 ;
vector<KeyPoint> imgpts1;
vector<KeyPoint> imgpts2;
vector<KeyPoint> imgpts1_good;
vector<KeyPoint> imgpts2_good;
vector<DMatch>   match ;

    Matx34d    P = cv::Matx34d(1,0,0,0,
						0,1,0,0,
						0,0,1,0);
	Matx34d    P1 = cv::Matx34d(1,0,0,50,
						 0,1,0,0,
						 0,0,1,0);

		int sp =  CalculateMotionMap(Iinp->img  , 
                               Iinp1->img, 
                       		   imgpts1 , 
                               imgpts2 ,
                               pt1,
                               pt2,
                               imgpts1_good,
                               imgpts2_good,
                               match);


	FindCameraMatrices( K, Kinv,  distortion_coeff,imgpts1, imgpts2, imgpts1_good,imgpts2_good,P,P1,
match, pointcloud);
cout<< "Hey "<< endl ; 

for (unsigned int i = 0 ; i < pointcloud.size(); i++){
    cout<<pointcloud[i].pt.x<<" , "<<pointcloud[i].pt.y<<" , "<<pointcloud[i].pt.z<<endl;
}


int size1 = pointcloud.size();
cout<<size1<<endl;
point *temppoint = new point[size1];


temppoint = Recoverpoint( pointcloud  , temppoint);
cout<<"Hey there"<<endl;
for (unsigned int i = 0 ; i < pointcloud.size(); i++){
    cout<<temppoint[i].P.x<<" , "<<temppoint[i].P.y<<" , "<<temppoint[i].P.z<<endl;
}


// temppoint = convertcloudPointtopoint(temppoint, pointcloud);
// cout <<"I am almost there"<<endl ;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_display(new pcl::PointCloud<pcl::PointXYZ> ());

//  cout<<"I will do this"<< endl;

point_display = convertintopointcloud(temppoint, point_display,pointcloud.size());


visualization(point_display);
	

pcl::PolygonMesh triangles;

triangles = pointcloudmesh(point_display);
//     // namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    
//     // imshow( "Contours", Iinp1->img);
  
//     // waitKey(0) ;

delete temppoint ; 
delete Iinp ; 
delete Iinp1;
K.release();
Kinv.release();
cam_matrix.release();
distortion_coeff.release();
return 0 ; 

}
