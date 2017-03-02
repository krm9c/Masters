
/*
Header Files
*/



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
1.> Function to read the image using the filename
*/

Mat Imagetomat(string filename); 



/*
2.> Acquisition of the image and storage on the disk
It was required in the system that all images coming from the frame grabber be stored in the disk. 
This function whenever called chooses an image from the stream of images and 
stores it on the disk for use of the user.
*/

ImageMat *ImageAcquisition(int choice ,string filename ,string key ,I_List &I_object );



/*
3.> Thresholding of the image according to RGB values. Separate values for R,G,and B can be used.
Also if in future, thresholding of coordinates according to proximity values needs to be done, then can be done.  
*/

ImageMat *thresholdRGB(ImageMat* Iparameter, coordinates Cthreshold, RGB RGBthreshold, string key, I_List I_object);



/*
4.> Conversion into Grayscale of a funciton 
*/

ImageMat *imageconvertgrayscale(ImageMat *I);



/*
5.> Prelimnary edge detection to be done before calculating the output contours of an object in the image
*/

ImageMat *PrelimEdgeDetection(ImageMat *Isrc, string key, I_List &I_object);



/*
6.> Store Motion calculated in the images. These images are stored in a associative array format.
*/

void storemotion(M_List &M, Matx34d P);




/*
7.> Retreive Motion calculated in the images. These images are stored in a associative array format.
*/

Matx34d retreivemotion(M_List &M, time_t t);



/*
8.> Retreive images calculated in the images. These images are stored in a associative array format.
*/

ImageMat retreiveimages( I_List &I , time_t t , string key);



/*
9.> Retreive lines calculated in the images. These images are stored in a associative array format.
*/

lines *retreivelines(L_List &L ,time_t t , string key);



/*
9.> Retreive lines calculated in the images. These images are stored in a associative array format.
*/

point retreivepoints(P_List &P, time_t t , string key);



/*
10.> Remove the time key from the list.
*/

void remove(vector<time_t> & v, time_t & item);



/*
11.> Remove the string key from the list.
*/

void remove(vector<string> & v, string & item);



/*
12.> Erase motion from the list.
*/

void erasemotion(M_List &M, time_t t );



/*
13.>  Erase points from the list.
*/

void erasepoints(P_List &P , time_t t , string key);



/*
14.>  Erase lines from the list.
*/

void eraselines(L_List &L , time_t t , string key);



/*
15.>  Erase images from the list.
*/

void eraseimages(I_List &I , time_t t , string key);



/*
16.>  Store points into the list.
*/

void  storepoints(P_List &P ,point p, string key );



/*
17.>  Store images into the list.
*/

void storeimage(I_List &I ,ImageMat M , string key );



/*
18.>  Display all string keys into a list.
*/

void Display_All_Keys_string(vector<string> L){



/*
19.>  Display all TIME keys into a list.
*/

void Display_All_Keys_time(vector<time_t> L);



/*
20.>  Compare the points to determine whether the points are same.
*/

int comparepoints(lines *temp,lines *temp1);



/*
21.>  Print all the lines in an image.
*/

void *printlines(lines *start);



/*
22.>  Convert all lines into points
*/

lines *convert(point *V , int numberofpointstored);



/*
23.>  Display all points on image
*/

Mat Display_Points_onImage(point *Vertexes , Mat im_bw1 , int numberofpointstored);



/*
24.>  Detect all contour points in an image 
*/

ImageMat *VerticesDetection(ImageMat *I , string key , P_List &P_ob , L_List &L_ob ,I_List &I_ob );



/*
25.>  Detect all contour points in an image 
*/

point *assignlabels(point *V);



/*
26.>  Create matches in a point
*/

void matches2points(const vector<KeyPoint>& train, 
                    const vector<KeyPoint>& query,
                    const vector<DMatch>& matches, 
                    vector<Point2f>& pts_train,
                    vector<Point2f>& pts_query);



/*
27.>  Convert points into Keypoints
*/

void PointsToKeypoints(const vector<Point2f>& in, vector<KeyPoint>& out);



/*
28.>  Convert cloudpoints into points
*/

std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts);



/*
29.>  Convert keypoints into points
*/

void KeypointsToPoints(const vector<KeyPoint>& in, vector<Point2f>& out);



/*
30.>  Get aligned points from Match
*/

void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2);



/*
31.> Take Scalar vector decomposition of Essential Matrix
*/

void TakeSVDOfE(Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w);



/*
32.> Take Scalar vector decomposition of Essential Matrix using a method described by HZ.
*/

bool DecomposeEtoRandT( Mat_<double>& E,
    Mat_<double>& R1,
    Mat_<double>& R2,
    Mat_<double>& t1,
    Mat_<double>& t2);



/*
33.> Check Coherant Rotation
*/

bool CheckCoherentRotation(cv::Mat_<double>& R) ;



/*
34.> Test Triangulation
*/

bool TestTriangulation(const vector<CloudPoint>& pcloud, const Matx34d& P, vector<uchar>& status);



/*
35.> Get fundamental matrix
*/

Mat GetFundamentalMat( const vector<KeyPoint>& imgpts1,
                       const vector<KeyPoint>& imgpts2,
                       vector<KeyPoint>& imgpts1_good,
                       vector<KeyPoint>& imgpts2_good,
                       vector<DMatch>& matches
                      );



/*
36.> Get fundamental matrix
*/
// Motion map use both feature extractor and optical flow .

int  CalculateMotionMap(Mat  img_1  , 
                        Mat   img_2 , 
                        vector<KeyPoint> &keypoints_1 , 
                        vector<KeyPoint> &keypoints_2 ,
                        vector<KeyPoint>& fullpts1,
                        vector<KeyPoint>& fullpts2,
                        vector<KeyPoint> &imgpts1_good,
                        vector<KeyPoint> &imgpts2_good,
                        vector <DMatch>  &matches);



/*
37.> Find camera matrices
*/

bool FindCameraMatrices(const Mat& K, 
                        const Mat& Kinv, 
                        const Mat& distcoeff,
                        const vector<KeyPoint>& imgpts1,
                        const vector<KeyPoint>& imgpts2,
                        vector<KeyPoint>& imgpts1_good,
                        vector<KeyPoint>& imgpts2_good,
                        Matx34d& P,
                        Matx34d& P1,
                        vector<DMatch>& matches,
                        vector<CloudPoint> &outCloud);



/*
38.> Linear traingualtion,  From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/

Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                                   Matx34d P,       //camera 1 matrix
                                   Point3d u1,      //homogenous image point in 2nd camera
                                   Matx34d P1       //camera 2 matrix
                                   );



/*
39.> Iterative linear traingualtion, // From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
*/

Mat_<double> IterativeLinearLSTriangulation(Point3d u,  //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
                                            );



/*
40.> Traingualte Points
*/

double TriangulatePoints(const vector<KeyPoint>& pt_set1, 
                        const vector<KeyPoint>& pt_set2, 
                        const Mat& K,
                        const Mat& Kinv,
                        const Mat& distcoeff,
                        const Matx34d& P,
                        const Matx34d& P1,
                        vector<CloudPoint>& pointcloud,
                        vector<KeyPoint>& correspImg1Pt);



/*
41.> Recover Point
*/

point *Recoverpoint( vector<CloudPoint>& pointcloud  , point *p);



/*
42.> Visualisation functions
*/

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



void visualization( pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);



pcl::PointCloud<pcl::PointXYZ>::Ptr convertintopointcloud(point *p , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,int size);



void imshow_250x250(const string& name_, const Mat& patch);



pcl::PolygonMesh pointcloudmesh( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);



