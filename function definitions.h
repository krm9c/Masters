# include "declarations.h"
/***************************************************************************
Function to store the image which is stored on the SSD on a matrix
function called directly from the ImageAcquisition. internally .
***************************************************************************/
Mat Imagetomat(string filename )
{   
    Mat img;
    img = imread(filename);   // read the image from the file using the filename
    if(! img.data)                              	  // Check for invalid input
    {
        cout <<  "Could not open or find the image Error!!!" << std::endl ;        
    }
    return img;
}

/******************************************************************************
   Function definitions for acquisition of data into the matrix. and into SSD
   choice=1 <Stores into SSD with the filename of your choice>
   choice=2 <Stores into matrix of your choice>
   if key='no ' then do not sotre the original image ,otherwise store it with the key passed as parameter. 
******************************************************************************/

ImageMat *ImageAcquisition(int choice , string filename , string key , I_List &I_object){
    if (choice == 1){
    ImagetoSSD(filename) ;// Call the function and read the file using the filename 
    }
    ImageMat *I = new ImageMat;// Declare a pointer this way 
    Mat imgtemp;
    imgtemp=Imagetomat(filename); // takes the image out of the filesystem and makes it usable by the code.
    I->img = imgtemp.clone() ;
    if (!key.compare("no"))
        storeimage(I_object, *I , key );
    imgtemp.release() ;    
    return  I;
}

/*******************************************************************************************
 Function to do the thresholding .
 RGB threshold according to separate RGB values .
 Cparameter is an object of coordinates having x,y ,z values
 Rt of RGB and coordinates
********************************************************************************************/   
ImageMat *thresholdRGB(ImageMat* Iparameter ,coordinates Cthreshold ,RGB RGBthreshold, string key,I_List I_object){
    Mat imgsrc ;
    imgsrc =Iparameter->img.clone();
    Mat channels[3]                ;
    split(imgsrc , channels)       ;

    // Thresholding Different channels
    threshold(channels[0], channels[0], (RGBthreshold.B*255),255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    threshold(channels[1], channels[1], (RGBthreshold.G*255),255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    threshold(channels[2], channels[2], (RGBthreshold.R*255),255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    merge(channels,3,imgsrc)       ;
    Iparameter->img=imgsrc.clone() ;
    if (!key.compare("no"))
         storeimage(I_object, *Iparameter , key );

    imgsrc.release()               ;
    channels[0].release()          ;
    channels[1].release()          ;
    channels[2].release()          ;
    return Iparameter              ;
}

/******************************************************************************************
Function to convert an image into grayscale.
*******************************************************************************************/
ImageMat *imageconvertgrayscale(ImageMat *I){
    Mat imgsrc                              ;
    cvtColor(I->img , imgsrc , CV_RGB2GRAY) ;  // Convert to grayscale
    I->img = imgsrc.clone()                 ;
    imgsrc.release()                        ;
    return I                                ;
}
/*****************************************************************************************    
// Function to do the edge detection
// Parameters are Image_Mat
// Usage:
          Iedge = prelimEdgedetection(Isrc) 

Iedge and Isrc are bothe of the type Image_Mat
*****************************************************************************************/
// Detecting a contour and putting all of them in a list of points
ImageMat *PrelimEdgeDetection(ImageMat *Isrc , string key , I_List &I_object){
    Mat im_bw ;
    cv::Mat newImage;
    Isrc=imageconvertgrayscale(Isrc);
    im_bw = Isrc->img.clone();
    threshold(im_bw , im_bw, 128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU)  ;
    cv::Mat mask;
    im_bw.copyTo(mask);

    // making  all the discontinuities in the image more precise and continuities rather dilating the edges to be more filled 
    for (int i = 0; i < mask.cols; i++) {
        if (mask.at<char>(0, i) == 255) {
            cv::floodFill(mask, cv::Point(i, 0), 0, 0, 10, 10);
        }
        if (mask.at<char>(mask.rows-1, i) == 255) {
            cv::floodFill(mask, cv::Point(i, mask.rows-1), 0 , 0, 10, 10);
        }
    }
    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<char>(i, 0) == 255) {
            cv::floodFill(mask, cv::Point(0, i), 0, 0, 10, 10);
        }
        if (mask.at<char>(i, mask.cols-1) == 255) {
            cv::floodFill(mask, cv::Point(mask.cols-1, i), 0 , 0, 10, 10);
        }
    }

    // Compare mask with original.
    im_bw.copyTo(newImage);
    for (int row = 0; row < mask.rows; ++row) {
        for (int col = 0; col < mask.cols; ++col) {
              if (mask.at<char>(row, col) == 255)     {
                newImage.at<char>(row, col) = 0;
            }
        }
    }

    // Histogram Equalisation and  Thresholding using Gaussian Blur .To make the edge more precise and visible
    cv::Mat gaussKernel = cv::getGaussianKernel(9,1.5,CV_32F);
    cv::GaussianBlur( newImage, im_bw , cv::Size(3,3), 1.5);
    double high_thres = 1.5 * cv::threshold( im_bw, im_bw, 0, 255,
                                            CV_THRESH_BINARY && CV_THRESH_OTSU );
    double LOW_THRES = 0.5 * high_thres ;
    
    // Edge detection
    Canny (im_bw , im_bw  , high_thres , LOW_THRES , 3 , false ) ;
    
    // Deleting all the memory assignments
    Isrc->img=im_bw.clone() ;
    imwrite("edge.jpg", Isrc->img);

    if (key.compare("no")!=0)
        storeimage(I_object, *Isrc,key);   
    
    im_bw.release() ;
    mask.release() ;
    newImage.release();
    return Isrc ;
}

/************************************************************************************************************
**************************************************************************************************************/
void storemotion(M_List &M , Matx34d P )
{
    time_t t = time(0);
    M.keytimemotion.push_back(t);
    M.TrackMotion[t]=P;
}

/************************************************************************************************************
**************************************************************************************************************/
Matx34d retreivemotion(M_List &M , time_t t){
return(M.TrackMotion[t]);
}
/************************************************************************************************************
**************************************************************************************************************/
ImageMat retreiveimages( I_List &I , time_t t , string key){
 return (I.Images_List[std::make_pair(key,t)]) ;
}
/************************************************************************************************************
**************************************************************************************************************/
lines *retreivelines(L_List &L ,time_t t , string key){
   return (L.Lines_List[std::make_pair(key,t)]) ;
}
/************************************************************************************************************
**************************************************************************************************************/
point retreivepoints(P_List &P, time_t t , string key){
  return (P.Points_List[std::make_pair(key,t)]) ; 
}
/************************************************************************************************************

**************************************************************************************************************/
void remove(vector<time_t> & v, time_t & item)
{
    vector<time_t>::iterator result = find(v.begin(), v.end(), item);
     if (result == v.end())
        cout << "That key is not in there!" << endl;
    else
        v.erase(result);
}
/************************************************************************************************************
**************************************************************************************************************/

void remove(vector<string> & v, string & item)
{
     vector<string>::iterator result = find(v.begin(), v.end(), item);
     if (result == v.end())
        cout << "That key is not in there!" << endl;
    else
        v.erase(result);
}
 
/************************************************************************************************************

**************************************************************************************************************/


void erasemotion(M_List &M, time_t t ){
M. TrackMotion.erase(t);
remove(M.keytimemotion,t); 
}


void erasepoints(P_List &P , time_t t , string key){
P.Points_List.erase(std::make_pair(key,t));
remove(P.keyouterPoints,key);
remove(P.keytimePoints,t); 
}
/************************************************************************************************************

**************************************************************************************************************/
void eraselines(L_List &L , time_t t , string key){
L.Lines_List.erase(std::make_pair(key,t));
remove(L.keyouterLines,key);
remove(L.keytimeLines,t); ;
//write code for the deletion 
}

/************************************************************************************************************

************************************************************************************************************/

void eraseimages(I_List &I , time_t t , string key){
I.Images_List.erase(std::make_pair(key,t)); 
remove(I.keyouterImages,key);
remove(I.keytimeImages ,t  ); 
// Write code for erasing it form the keys storage.
}

/**************************************************************************************************************

***************************************************************************************************************/

void  storelines(L_List &L , lines *l , string key ){
    time_t t = time(0);
    L.keyouterLines.push_back(key);
    L.keytimeLines.push_back(t)  ;
    L.Lines_List[std::make_pair(key,t)]=l;
 }

/**************************************************************************************************************

**************************************************************************************************************/

void  storepoints(P_List &P ,point p, string key ){
    time_t t = p.t;
    P.keyouterPoints.push_back(key);
    P.keytimePoints.push_back(t)  ;
    P.Points_List[std::make_pair(key,t)]=p;
}
/*****************************************************************************************************************

/*****************************************************************************************************************/
void storeimage(I_List &I ,ImageMat M , string key ){
    time_t t = time(0);
    I.keyouterImages.push_back(key);
    I.keytimeImages.push_back(t)   ;
    I.Images_List[std::make_pair(key,t)]=M;
}
/*****************************************************************************************************************

*****************************************************************************************************************/
void Display_All_Keys_string(vector<string> L){
vector<string>::const_iterator it;
cout<< "Display all the keys in the List"<< endl ; 
for(it=L.begin(); it!=L.end(); ++it)
{
 cout << *it << endl; // each element on a separate line
} 
}
/*****************************************************************************************************************

*****************************************************************************************************************/

void Display_All_Keys_time(vector<time_t> L){
vector<time_t>::const_iterator it;
cout<< "Displaying all the keys in the List"<< endl ; 
for(it=L.begin(); it!=L.end(); ++it)
{
 cout << *it << endl; // each element on a separate line
} 
}

/*****************************************************************************************************************

*****************************************************************************************************************/
int comparepoints(lines *temp,lines *temp1)
{
    if ((temp->vertex.P.x ==temp1->vertex.P.x)&&(temp->vertex.P.x ==temp1->vertex.P.x)){
        return 0 ; 
    }
    else
        return 1 ; 

}

/******************************************************************************************************************

*******************************************************************************************************************/

void *printlines(lines *start){
lines *temp = start;
lines *temp1=temp;
while (comparepoints(temp,temp1)!=0){
    cout<< "x= "<<temp1->vertex.P.x<< "y="<<temp1->vertex.P.y<<endl ; 
    temp1=temp1->next;
    }
}

/***********************************************************************************************************************

/***********************************************************************************************************************/

lines *convert(point *V , int numberofpointstored){
// break into a separate function once it works 
    lines *temp =new lines;
    lines *start=new lines;
    for (int i=0 ; i<numberofpointstored;i ++){
        if (i==0){
            start->vertex=V[0];
            start->next=new lines;
            temp=start;
        }
        else if(i == numberofpointstored-1){
                temp->next=start;
            } 
        else{
            temp->vertex=V[i] ;
            temp->next = new lines;
        }
        temp=temp->next ;
    }
}

/***********************************************************************************************************************


***********************************************************************************************************************/


Mat Display_Points_onImage(point *Vertexes , Mat im_bw1 , int numberofpointstored){
    Point2f V1[100];
    for (int i = 0 ; i < numberofpointstored ; i ++){
        V1[i].x=Vertexes[i].P.x;
        V1[i].y=Vertexes[i].P.y;
           
    }

    for (int i = 0 ;i<numberofpointstored ;i++){
    //circle(im_bw1, Point(V1[i].x,V1[i].y) ,50, Scalar(0,0,0),CV_FILLED, 8,0);
    if ((i+1)<numberofpointstored)
        line(im_bw1, V1[i], V1[i+1],Scalar(0,0,0), 1, 8, 0);
    else
        line(im_bw1, V1[i], V1[0],Scalar(0,0,0), 1, 8, 0);
    }

    return im_bw1;
}
/***********************************************************************************************************************

************************************************************************************************************************/
ImageMat *VerticesDetection(ImageMat *I , string key , P_List &P_ob , L_List &L_ob ,I_List &I_ob ){
    Mat im_bw=I->img.clone() ;
    Mat drawing = Mat::zeros( im_bw.size(), CV_8UC3 );
    point *V=new point[100];
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( im_bw , contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE , Point(0, 0) );
    float ratio ;
    int largest_contour_index=0 , largest_area= 0  ;
    Rect bounding_rect;
    double a ;
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    for( int i = 0; i < contours.size(); i++ ) {
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 5, true);
    }
    // merge all contours into one vector
    std::vector<cv::Point> merged_contour_points;
    for (int i = 0; i < contours_poly.size(); i++) {
        for (int j = 0; j < contours_poly[i].size(); j++) {
            merged_contour_points.push_back(contours_poly[i][j]);
        }
    }
    // get rotated bounding box
    std::vector<cv::Point> hull;
    cv::convexHull(cv::Mat(merged_contour_points),hull);
    cv::Mat hull_points(hull);
    cv::RotatedRect rotated_bounding_rect = minAreaRect(hull_points);
    Point2f vertices[4];
    rotated_bounding_rect.points(vertices);    
    //   for (int i = 0; i < 4; i++)
    //   line(im_bw, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
    time_t timecurrent = time(0);
    for (int i = 0 ; i < hull.size() ; i ++){
        V[i].t =timecurrent ;
        V[i].P.x=hull[i].x ;
        V[i].P.y=hull[i].y ;
        V[i].C.B=I->img.at<cv::Vec3b>(hull[i].y,hull[i].x)[0] ;
        V[i].C.G=I->img.at<cv::Vec3b>(hull[i].y,hull[i].x)[1] ;
        V[i].C.R=I->img.at<cv::Vec3b>(hull[i].y,hull[i].x)[2] ;
    }

    Mat im_bw1= im_bw.clone();
    im_bw1=Scalar(255,255,255);
    cout << "Hull size is "<< hull.size()<<endl; 
 
    I->number_points_detected = hull.size(); 

    lines *structure=convert(V , I->number_points_detected);

    im_bw1=Display_Points_onImage(V,im_bw1 , I->number_points_detected );

    // break this display function once it starts working 

    I->img = im_bw1.clone();
    imwrite("wireframe.jpg", im_bw1);
    cout <<"key in the compare"<< key << endl ;
     if (key.compare("no")!=0)
         storeimage(I_ob, *I , key );
    assignlabels(V);
     for (int i = 0 ; i < hull.size(); i++){
        key = V[i].label;
        storepoints(P_ob, V[i] , key );
    }
    if (key.compare("no")!=0)
        storelines(L_ob, structure , key );

    drawing.release()     ;
    im_bw1.release()      ;
    im_bw.release()       ;
    return I              ; 
}
/***********************************************************************************************************************

************************************************************************************************************************/

point *assignlabels(point *V){
    vector<point>::iterator it;
    int dist = 0 ; 
    int flag =1;
    point ptemp  ;
    long int lowestdistance=1000000;
    int index= 0 ; 
    if((!coordinates_list.empty())){
        for (int i= 0 ; i <sizeof(V);i++){
            for (it = coordinates_list.begin();it!=coordinates_list.end();++it){
                dist = sqrt((V[i].P.x-it->P.x)*(V[i].P.x-it->P.x)+(V[i].P.y-it->P.y)*(V[i].P.y-it->P.y)) ;
                if (dist < lowestdistance){
                    dist=lowestdistance ;
                    ptemp=*it;
                    flag=0;
                }
            }
            if (flag==0){
                V[i].label = ptemp.label;
                flag=2;
            }
            else
                V[i].label=labelno++;
        }

    }
    else{
        for (int i = 0 ; i < sizeof(V); i ++)
            V[i].label=labelno++;
   }
    
    for (int i = 0 ; i < sizeof(V); i ++){
            coordinates_list.push_back(V[i]);
    }
    return V ;
}

/***********************************************************************************************************************


************************************************************************************************************************/

/***********************************************************************************************************************
query  is the image which is being matches and train is the one to which the matching is done . Convwntional naming .
************************************************************************************************************************/

void matches2points(const vector<KeyPoint>& train, 
                    const vector<KeyPoint>& query,
                    const vector<DMatch>& matches, 
                    vector<Point2f>& pts_train,
                    vector<Point2f>& pts_query) {

// Clear all the vector classes
  pts_train.clear();
  pts_query.clear();
  pts_train.reserve(matches.size());
  pts_query.reserve(matches.size());
 //push into points 
  size_t i = 0;
  for (; i < matches.size(); i++) {
    const DMatch & dmatch = matches[i];
    pts_query.push_back(query[dmatch.queryIdx].pt);
    pts_train.push_back(train[dmatch.trainIdx].pt);
  }
}


/***********************************************************************************************************************
keypoints consists of points with some RGB characteristic values and consist of the neighbourhood of the points thiuus detected on the image
so we just change it to a point which is understood by us easily next two functions do just that. The word keypoint is standard 
convention with opencv and we change it whenever we use it . 
************************************************************************************************************************/

void PointsToKeypoints(const vector<Point2f>& in, vector<KeyPoint>& out){
  out.clear();
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i)
    out.push_back(KeyPoint(in[i], 1));
}

/***********************************************************************************************************************

************************************************************************************************************************/


std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts) {
    std::vector<cv::Point3d> out;
    for (unsigned int i=0; i<cpts.size(); i++) {
        out.push_back(cpts[i].pt);
    }
    return out;
}



void KeypointsToPoints(const vector<KeyPoint>& in, vector<Point2f>& out){
  out.clear();
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i)
    out.push_back(in[i].pt);
}




/***********************************************************************************************************************
When we find match we still need to align them with each other so that further processing can be found and done over there 

Again going with the convention and using query and train to explain the various details 

************************************************************************************************************************/
void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
                               const std::vector<cv::KeyPoint>& imgpts2,
                               const std::vector<cv::DMatch>& matches,
                               std::vector<cv::KeyPoint>& pt_set1,
                               std::vector<cv::KeyPoint>& pt_set2) 
{
    for (unsigned int i=0; i<matches.size(); i++) {
        pt_set1.push_back(imgpts1[matches[i].queryIdx]);
        pt_set2.push_back(imgpts2[matches[i].trainIdx]);
    }   
}


/***********************************************************************************************************************
// We use SVD to get our essential matrix to convert itself into much smaller portions 
//Scalar vector Decomposition 
/************************************************************************************/

void TakeSVDOfE(Mat_<double>& E, Mat& svd_u, Mat& svd_vt, Mat& svd_w) {
    //Using OpenCV's SVD
    SVD svd(E,SVD::MODIFY_A);
    svd_u = svd.u;
    svd_vt = svd.vt;
    svd_w = svd.w;
    cout << "----------------------- SVD ------------------------\n";
    cout << "U:\n"<<svd_u<<"\nW:\n"<<svd_w<<"\nVt:\n"<<svd_vt<<endl;
    cout << "----------------------------------------------------\n";
}



/***********************************************************************************************************************

************************************************************************************************************************/


/************************************************************************************/



// bool DecomposeEssentialUsingHorn90(double _E[9], double _R1[9], double _R2[9], double _t1[3], double _t2[3]) {

// #ifdef USE_EIGEN
//     using namespace Eigen;

//     Matrix3d E = Map<Matrix<double,3,3,RowMajor> >(_E);
//     Matrix3d EEt = E * E.transpose();
//     Vector3d e0e1 = E.col(0).cross(E.col(1)),e1e2 = E.col(1).cross(E.col(2)),e2e0 = E.col(2).cross(E.col(2));
//     Vector3d b1,b2;


//     Matrix3d bbt = 0.5 * EEt.trace() * Matrix3d::Identity() - EEt; //Horn90 (12)
//     Vector3d bbt_diag = bbt.diagonal();
//     if (bbt_diag(0) > bbt_diag(1) && bbt_diag(0) > bbt_diag(2)) {
//         b1 = bbt.row(0) / sqrt(bbt_diag(0));
//         b2 = -b1;
//     } else if (bbt_diag(1) > bbt_diag(0) && bbt_diag(1) > bbt_diag(2)) {
//         b1 = bbt.row(1) / sqrt(bbt_diag(1));
//         b2 = -b1;
//     } else {
//         b1 = bbt.row(2) / sqrt(bbt_diag(2));
//         b2 = -b1;
//     }
//     // Find out what is HORN 90 method .
//     //Horn90 (19)
//     Matrix3d cofactors; cofactors.col(0) = e1e2; cofactors.col(1) = e2e0; cofactors.col(2) = e0e1;
//     cofactors.transposeInPlace();
    
//     //B = [b]_x , see Horn90 (6) and http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
//     Matrix3d B1; B1 <<  0,-b1(2),b1(1),
//                         b1(2),0,-b1(0),
//                         -b1(1),b1(0),0;
//     Matrix3d B2; B2 <<  0,-b2(2),b2(1),
//                         b2(2),0,-b2(0),
//                         -b2(1),b2(0),0;

//     Map<Matrix<double,3,3,RowMajor> > R1(_R1),R2(_R2);

//     //Horn90 (24)
//     R1 = (cofactors.transpose() - B1*E) / b1.dot(b1);
//     R2 = (cofactors.transpose() - B2*E) / b2.dot(b2);
//     Map<Vector3d> t1(_t1),t2(_t2); 
//     t1 = b1; t2 = b2;
    
//     cout << "Horn90 provided " << endl << R1 << endl << "and" << endl << R2 << endl;

// }

/***********************************************************************************

 hARTLEY AND Ziserman shows us how to use SVD to decompose E and then multiply with an arbitrary matrix w to get the \
  R & T .
************************************************************************************/

bool DecomposeEtoRandT( Mat_<double>& E,
    Mat_<double>& R1,
    Mat_<double>& R2,
    Mat_<double>& t1,
    Mat_<double>& t2) 
{
    //Using HZ E decomposition
    // basically take svd of E and return all teh matrices and then multiply it with an arbitrary matrix W and we get the R &T 

    Mat svd_u, svd_vt, svd_w;
    TakeSVDOfE(E,svd_u,svd_vt,svd_w);

    //check if first and second singular values are the same (as they should be)

    double singular_values_ratio = fabsf(svd_w.at<double>(0) / svd_w.at<double>(1));
    if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
    if (singular_values_ratio < 0.7) {
        cout << "singular values are too far apart\n";
        return false;
    }


    // Declaring the arbitrary matrix , The matrix is defined by HZ .

    Matx33d W(0,-1,0,   //HZ 9.13
        1,0,0,
        0,0,1);
    Matx33d Wt(0,1,0,
        -1,0,0,
        0,0,1);

    R1 = svd_u * Mat(W) * svd_vt; //HZ 9.19
    R2 = svd_u * Mat(Wt) * svd_vt; //HZ 9.19
    t1 = svd_u.col(2); //u3
    t2 = -svd_u.col(2); //u3

    return true;
}
/***********************************************************************************


************************************************************************************/



/************************************************************************************/


/***********************************************************************************


************************************************************************************/



bool CheckCoherentRotation(cv::Mat_<double>& R) {

    
    if(fabsf(determinant(R))-1.0 > 1e-07) {
        cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
        return false;
    }

    return true;
}



/***********************************************************************************



************************************************************************************/

bool TestTriangulation(const vector<CloudPoint>& pcloud, const Matx34d& P, vector<uchar>& status) {
    
    //Declare the point clouds . they are  nothing but a vector of 3d structure
    
    vector<Point3d> pcloud_points3d  = CloudPointsToPoints(pcloud);
    vector<Point3d> pcloud_point3d_projected(pcloud_points3d.size());

    Matx44d P4x4 = Matx44d::eye(); 

    for(int i=0;i<12;i++) 
        P4x4.val[i] = P.val[i];
    perspectiveTransform(pcloud_points3d, pcloud_point3d_projected, P4x4);
    status.resize(pcloud.size(),0);

    for (int i=0; i<pcloud.size(); i++) {
        status[i] = (pcloud_point3d_projected[i].z > 0) ? 1 : 0;
    }

    int count = countNonZero(status);

    double percentage = ((double)count / (double)pcloud.size());
    
    cout << count << "/" << pcloud.size() << " = " << percentage*100.0 << "% are in front of camera" << endl;
    
    if(percentage < 0.75)
        return false; //less than 75% of the points are in front of the camera
//check for coplanarity of points

    if(false) //not
    {
        cv::Mat_<double> cldm(pcloud.size(),3);
        for(unsigned int i=0;i<pcloud.size();i++) {
            cldm.row(i)(0) = pcloud[i].pt.x;
            cldm.row(i)(1) = pcloud[i].pt.y;
            cldm.row(i)(2) = pcloud[i].pt.z;
        }

        cv::Mat_<double> mean;
// Principle Component Analysis
        cv::PCA pca(cldm,mean,CV_PCA_DATA_AS_ROW);

        int num_inliers = 0;
        cv::Vec3d nrm = pca.eigenvectors.row(2); nrm = nrm / norm(nrm);
        cv::Vec3d x0 = pca.mean;
        double p_to_plane_thresh = sqrt(pca.eigenvalues.at<double>(2));


        for (int i=0; i<pcloud.size(); i++) {
            Vec3d w = Vec3d(pcloud[i].pt) - x0;
            double D = fabs(nrm.dot(w));
            if(D < p_to_plane_thresh) num_inliers++;
        }

        cout << num_inliers << "/" << pcloud.size() << " are coplanar" << endl;

        if((double)num_inliers / (double)(pcloud.size()) > 0.85)
            return false;
    }
    
    return true;
}



point *convertcloudPointtopoint(point *temppoint , vector<CloudPoint> outCloud){
for (int i = 0 ; i < outCloud.size();i++){
    outCloud[i].pt.x = temppoint[i].P.x;
    outCloud[i].pt.y = temppoint[i].P.y;
    outCloud[i].pt.z = temppoint[i].P.z;
}
return temppoint ;
}



/**********************************************************************************************************



***********************************************************************************************************/

Mat GetFundamentalMat( const vector<KeyPoint>& imgpts1,
                       const vector<KeyPoint>& imgpts2,
                       vector<KeyPoint>& imgpts1_good,
                       vector<KeyPoint>& imgpts2_good,
                       vector<DMatch>& matches
                      ) 
{
    //Try to eliminate keypoints based on the fundamental matrix
    //(although this is not the proper way to do this)
    vector<uchar> status(imgpts1.size());
   
    //  undistortPoints(imgpts1, imgpts1, cam_matrix, distortion_coeff);
    //  undistortPoints(imgpts2, imgpts2, cam_matrix, distortion_coeff);
    //
    imgpts1_good.clear(); imgpts2_good.clear();
    
    vector<KeyPoint> imgpts1_tmp;
    vector<KeyPoint> imgpts2_tmp;
    if (matches.size() <= 0) {
        imgpts1_tmp = imgpts1;
        imgpts2_tmp = imgpts2;
    } else {
        GetAlignedPointsFromMatch(imgpts1, imgpts2, matches, imgpts1_tmp, imgpts2_tmp);
    }
    Mat F;
    vector<Point2f> pts1,pts2;
    KeypointsToPoints(imgpts1_tmp, pts1);
    KeypointsToPoints(imgpts2_tmp, pts2);
    double minVal,maxVal;
    cv::minMaxIdx(pts1,&minVal,&maxVal);
    F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.006 * maxVal, 0.99, status); //threshold from [Snavely07 4.1]
    vector<DMatch> new_matches;
    cout << "F keeping " << countNonZero(status) << " / " << status.size() << endl; 
    for (unsigned int i=0; i<status.size(); i++) {
        if (status[i]) 
        {
            imgpts1_good.push_back(imgpts1_tmp[i]);
            imgpts2_good.push_back(imgpts2_tmp[i]);
            //new_matches.push_back(DMatch(matches[i].queryIdx,matches[i].trainIdx,matches[i].distance));
            new_matches.push_back(matches[i]);

        }
    }   
    
    cout << matches.size() << " matches before, " << new_matches.size() << " new matches after Fundamental Matrix\n";
    matches = new_matches; //keep only those points who survived the fundamental matrix

    
    return F;
}

/*************************************************************************************************************

**************************************************************************************************************/


// Motion map use both feature extractor and optical flow .
int  CalculateMotionMap(Mat  img_1  , 
                        Mat   img_2 , 
                        vector<KeyPoint> &keypoints_1 , 
                        vector<KeyPoint> &keypoints_2 ,
                        vector<KeyPoint>& fullpts1,
                        vector<KeyPoint>& fullpts2,
                        vector<KeyPoint> &imgpts1_good,
                        vector<KeyPoint> &imgpts2_good,
                        vector <DMatch>  &matches){

     Mat descriptors_1 ;
                        Mat descriptors_2 ; 
        //-- Step 1: Detect the keypoints using SURF Detector
        int minHessian = 400;
        
        //      GridAdaptedFeatureDetector detector(new SurfFeatureDetector(minHessian), 1000,1,1);
        SurfFeatureDetector detector( minHessian );
        
        
            detector.detect( img_1, keypoints_1 );
   
    
            detector.detect( img_2, keypoints_2 );
     
        
        //-- Step 2: Calculate descriptors (feature vectors)
        //      SurfDescriptorExtractor extractor(8,4,true);
        SiftDescriptorExtractor extractor(48,16,true);
        //  OpponentColorDescriptorExtractor extractor(new SurfDescriptorExtractor);
    
        extractor.compute( img_1, keypoints_1, descriptors_1 );

        extractor.compute( img_2, keypoints_2, descriptors_2 );       

  // Mat img_keypoints_1, img_keypoints_2 ;

  // drawKeypoints( img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
  // drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );



  // //-- Show detected (drawn) keypoints
  // imshow("Keypoints 1", img_keypoints_1 );
  // imshow("Keypoints 2", img_keypoints_2 );



//-- Step 3: Matching descriptor vectors using FLANN matcher
        //FlannBasedMatcher matcher;
        BFMatcher matcher(NORM_L2,true); //use an alternative to the ratio test
        std::vector< DMatch > good_matches_;
        
            cout << "matching desc1="<<descriptors_1.rows<<", desc2="<<descriptors_2.rows<<endl;
            matcher.match( descriptors_1, descriptors_2, matches );
    
        
        double max_dist = 0; double min_dist = 1000.0;
        //-- Quick calculation of max and min distances between keypoints
        for(unsigned int i = 0; i < matches.size(); i++ )
        { 
            double dist = (matches)[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        

        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );

        


        if (min_dist <= 0) {
            min_dist = 10.0;
        }
        
        double cutoff = 4.0*min_dist;
        std::set<int> existing_trainIdx;
        for(unsigned int i = 0; i < matches.size(); i++ )
        { 
            if ((matches)[i].trainIdx <= 0) {
                cout<<"I am everywhere"<<endl ; 
                (matches)[i].trainIdx = (matches)[i].imgIdx;
            }
            
            if( existing_trainIdx.find((matches)[i].trainIdx) == existing_trainIdx.end() && 
                (matches)[i].trainIdx >= 0 && (matches)[i].trainIdx < (int)(keypoints_2.size()) &&
                (matches)[i].distance > 0.0 && (matches)[i].distance < cutoff ) 
            {
                cout<< "I am here and i am everywhere"<< endl ;
                good_matches_.push_back( (matches)[i]);
                imgpts1_good.push_back(keypoints_1[(matches)[i].queryIdx]);
                imgpts2_good.push_back(keypoints_2[(matches)[i].trainIdx]);
                existing_trainIdx.insert((matches)[i].trainIdx);
            }
        }

//         cout << "keypoints_1.size() " << keypoints_1.size() << " imgpts1_good.size() " << imgpts1_good.size() << endl;
//         cout << "keypoints_2.size() " << keypoints_2.size() << " imgpts2_good.size() " << imgpts2_good.size() << endl;

//         {
//             //-- Draw only "good" matches


            if (good_matches_.size() ==0 ){
                    cout<< "No good matches where found moving on "<<endl ;
                    return 0 ;
                }

            Mat img_matches;
            drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                        good_matches_ , img_matches, Scalar::all(-1), Scalar::all(-1),
                        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );     
            //-- Show detected matches
       	imwrite("features.jpg", img_matches);
        vector<uchar> status;
        vector<KeyPoint> imgpts2_very_good,imgpts1_very_good;
        Mat outputflow ;
        Mat_<Point2f> flow_from_features(img_1.size());
        
        //Select features that make epipolar sense
        
        vector<Point2f> pts1,pts2;
        KeypointsToPoints(imgpts1_good, pts1);
        KeypointsToPoints(imgpts2_good, pts2);

    if ((imgpts1_good .size() ==0 ) && (imgpts2_good.size()==0)){
        cout << "Not enough points found !!!!   Abort !!!! Abort !!!!!! "<< endl ; 
            return 0 ; 
    }

    if ((pts1.size() ==0 ) && (pts2.size()==0)){
        cout << "Not enough points found !!!!   Abort !!!! Abort !!!!!! "<< endl ; 
            return 0 ; 
    }
    
    Mat F = findFundamentalMat(pts1, pts2, FM_RANSAC, 0.1, 0.99, status);
    
    cout << "Fundamental mat is keeping " << countNonZero(status) << " / " << status.size() << endl;    
    double status_nz = countNonZero(status); 
    double status_sz = status.size();
    double kept_ratio = status_nz / status_sz;
        
        if (kept_ratio > 0.2) {
            for (unsigned int i=0; i<imgpts1_good.size(); i++) {
                if (status[i]) 
                {
                    imgpts1_very_good.push_back(imgpts1_good[i]);
                    imgpts2_very_good.push_back(imgpts2_good[i]);
                }
            }
        }

        else{
            cout<<"Fundamental Matrix found is not good moving on to the next set of images "<< endl; 
            return 0; 
        }
                Mat_<double> T;
                {
                    vector<Point2f> pts1,pts2;
                    KeypointsToPoints(imgpts1_very_good, pts1);
                    KeypointsToPoints(imgpts2_very_good, pts2);

                    cout << "pts1 " << pts1.size() << endl;
                    cout << "pts2 " << pts2.size() << endl;
                     T = estimateRigidTransform(pts1,pts2, false);

                    cout << "rigid transform from features " << endl << T << endl;

                }

                cout<<"Got the rigig transform"<< endl ; 
                //Create the approximate flow using the estimated overall motion
                for (int x=0; x<img_1.cols; x++) {
                    for (int y=0; y<img_1.rows; y++) {
        //              Mat_<double> moved = H * (Mat_<double>(3,1) << x , y , 1);
                        Mat_<double> moved = T * (Mat_<double>(3,1) << x , y , 1);
                        Point2f movedpt(moved(0),moved(1));
                        flow_from_features(y,x) = Point2f(movedpt.x-x,movedpt.y-y);
                    }
                }

            cout << "Optical Flow..."<<endl;
     
            cout << "Dense..."<<endl;
            Mat_<Point2f> _flow , flow ;

            img_1.copyTo(outputflow);
            flow_from_features.copyTo(flow);
            //cvtColor(img_1, img_1, CV_BGR2GRAY);
            //cvtColor(img_2, img_2, CV_BGR2GRAY);

                  cout<<"Size of Flow "<< flow.size()<<" : "<< flow_from_features .size() <<endl ;
            //refine
            calcOpticalFlowFarneback(img_1,img_2,flow,0.5,2,40,40,5,0.5,OPTFLOW_USE_INITIAL_FLOW);
            calcOpticalFlowFarneback(img_1,img_2,flow,0.5,0,25,40,3,0.25,OPTFLOW_USE_INITIAL_FLOW);

            //imgpts1.clear(); imgpts2.clear(); 

            good_matches_.clear(); keypoints_1.clear(); keypoints_2.clear();

            for (int x=0;x<flow.cols; x+=1) {
                for (int y=0; y<flow.rows; y+=1) {
                    if (norm(flow(y,x)) < 20 || norm(flow(y,x)) > 100) {
                        continue; //discard points that havn't moved
                    }
                    Point2f p(x,y),p1(x+flow(y,x).x,y+flow(y,x).y);
                    //line(outputflow, p, p1, Scalar(0,255*norm(flow(y,x))/50), 1)
                    circle(outputflow, p, 1, Scalar(0,255*norm(flow(y,x))/50), 1);
                    if (x%10 == 0 && y%10 == 0) {
//                      imgpts1.push_back(KeyPoint(p,1));
//                      imgpts2.push_back(KeyPoint(p1,1));
                        good_matches_.push_back(DMatch(keypoints_1.size()-1,keypoints_1.size()-1,1.0));
                        keypoints_1.push_back(KeyPoint(p,1));
                        keypoints_2.push_back(KeyPoint(p1,1));
                    }
                    fullpts1.push_back(KeyPoint(p,1));
                    fullpts2.push_back(KeyPoint(p1,1));
                }
            }       

        cout <<"Points size"<< fullpts1.size()<<" , "<< fullpts2.size()<< endl ;
        
        imwrite("flow.jpg", outputflow);
    
        cout<< "Displayed"<< endl ; 
}




/***********************************************************************************
// Till here good code !!!
/***********************************************************************************


The basicx algorithm of this gives 4 values of camera matrix out of which three should be rejected .


P is assumed to be the reference point and hence is Identity . 

But P1 is calculated . 
esulting P1 is trhe camera rotation and transformation 


************************************************************************************/

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
                        vector<CloudPoint> &outCloud
                        ) 
{ //Find camera matrices
        cout << "Find camera matrices...";
        // We need atleast 100 matches in the images to 
        Mat F = GetFundamentalMat(imgpts1,imgpts2,imgpts1_good,imgpts2_good,matches );
        
        // if(matches.size() < 100) { // || ((double)imgpts1_good.size() / (double)imgpts1.size()) < 0.25
        //     cerr << "not enough inliers after F matrix !! abort " << endl;
        //     return false;
        // }


        //Essential matrix: compute then extract cameras [R|t]
        Mat_<double> E = K.t() * F * K;   // Refer Hz 

// Shiykd have a reduced rank 
        if(fabsf(determinant(E)) > 1e-07) {
            cout << "det(E) != 0 : " << determinant(E) << "\n";
            P1 = 0;
            return false;
        }


        Mat_<double> R1(3,3);
        Mat_<double> R2(3,3);
        Mat_<double> t1(1,3);
        Mat_<double> t2(1,3);
        //decompose E to P' , HZ (9.19)  
        if (!DecomposeEtoRandT(E,R1,R2,t1,t2)) 
            return false;

          if(determinant(R1)+1.0 < 1e-09) {
                //according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
                cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" << endl;
                E = -E;
                DecomposeEtoRandT(E,R1,R2,t1,t2);
            }
        if (!CheckCoherentRotation(R1)) {
                cout << "resulting rotation is not coherent\n";
                P1 = 0;
                return false;
        }
            
        P1 = Matx34d(R1(0,0),   R1(0,1),    R1(0,2),    t1(0),
                         R1(1,0),   R1(1,1),    R1(1,2),    t1(1),
                         R1(2,0),   R1(2,1),    R1(2,2),    t1(2));
        cout << "Testing P1 " << endl << Mat(P1) << endl;

            vector<CloudPoint> pcloud,pcloud1; vector<KeyPoint> corresp;
            double reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud, corresp);
            double reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);
            vector<uchar> tmp_status;
            //check if pointa are triangulated --in front-- of cameras for all 4 ambiguations
            if (!TestTriangulation(pcloud,P1,tmp_status) || !TestTriangulation(pcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                P1 = Matx34d(R1(0,0),   R1(0,1),    R1(0,2),    t2(0),
                             R1(1,0),   R1(1,1),    R1(1,2),    t2(1),
                             R1(2,0),   R1(2,1),    R1(2,2),    t2(2));
                cout << "Testing P1 "<< endl << Mat(P1) << endl;

                pcloud.clear(); pcloud1.clear(); corresp.clear();
                reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud, corresp);
                reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);
                
                if (!TestTriangulation(pcloud,P1,tmp_status) || !TestTriangulation(pcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                    if (!CheckCoherentRotation(R2)) {
                        cout << "resulting rotation is not coherent\n";
                        P1 = 0;
                        return false;
                    } 
                    P1 = Matx34d(R2(0,0),   R2(0,1),    R2(0,2),    t1(0),
                                 R2(1,0),   R2(1,1),    R2(1,2),    t1(1),
                                 R2(2,0),   R2(2,1),    R2(2,2),    t1(2));
                    cout << "Testing P1 "<< endl << Mat(P1) << endl;

                    pcloud.clear(); pcloud1.clear(); corresp.clear();
                    reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud, corresp);
                    reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);
                    
                    if (!TestTriangulation(pcloud,P1,tmp_status) || !TestTriangulation(pcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                        P1 = Matx34d(R2(0,0),   R2(0,1),    R2(0,2),    t2(0),
                                     R2(1,0),   R2(1,1),    R2(1,2),    t2(1),
                                     R2(2,0),   R2(2,1),    R2(2,2),    t2(2));
                        cout << "Testing P1 "<< endl << Mat(P1) << endl;

                        pcloud.clear(); pcloud1.clear(); corresp.clear();
                        reproj_error1 = TriangulatePoints(imgpts1_good, imgpts2_good, K, Kinv, distcoeff, P, P1, pcloud, corresp);
                        reproj_error2 = TriangulatePoints(imgpts2_good, imgpts1_good, K, Kinv, distcoeff, P1, P, pcloud1, corresp);
                        
                        if (!TestTriangulation(pcloud,P1,tmp_status) || !TestTriangulation(pcloud1,P,tmp_status) || reproj_error1 > 100.0 || reproj_error2 > 100.0) {
                            cout << "Shit." << endl; 
                            return false;
                        }
                    }               
                }           
            }
         for (unsigned int i=0; i<pcloud.size(); i++) {
                outCloud.push_back(pcloud[i]);
            }

            cout<<"I finished all the things and i calculated the point cloud"<<endl ;
      //  t = ((double)getTickCount() - t)/getTickFrequency();
       // cout << "Done. (" << t <<"s)"<< endl;
}
/***********************************************************************************


************************************************************************************/












/***********************************************************************************


************************************************************************************/



/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                                   Matx34d P,       //camera 1 matrix
                                   Point3d u1,      //homogenous image point in 2nd camera
                                   Matx34d P1       //camera 2 matrix
                                   ) 
{
    Matx43d A(u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),      
              u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),      
              u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),   
              u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    Matx41d B(-(u.x*P(2,3)  -P(0,3)),
              -(u.y*P(2,3)  -P(1,3)),
              -(u1.x*P1(2,3)    -P1(0,3)),
              -(u1.y*P1(2,3)    -P1(1,3)));
    Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);
    return X;
}

/************************************************************************************************/
/**




 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

 ************************************************************************************************/

Mat_<double> IterativeLinearLSTriangulation(Point3d u,  //homogenous image point (u,v,1)
                                            Matx34d P,          //camera 1 matrix
                                            Point3d u1,         //homogenous image point in 2nd camera
                                            Matx34d P1          //camera 2 matrix
                                            ) {
    double wi = 1, wi1 = 1;
    Mat_<double> X(4,1); 

    // loop to do iterative triangulation.

    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = LinearLSTriangulation(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
        
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);
        
        //breaking point
        if(fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;
        wi = p2x;
        wi1 = p2x1;
        //reweight equations and solve
        Matx43d A((u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,     
                  (u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,     
                  (u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1, 
                  (u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1
                  );
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)  -P(0,3))/wi,
                                                  -(u.y*P(2,3)  -P(1,3))/wi,
                                                  -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                                                  -(u1.y*P1(2,3)    -P1(1,3))/wi1);
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }

    return X;
}




/********************************************************************************************/

//Triagulate points
double TriangulatePoints(const vector<KeyPoint>& pt_set1, 
                        const vector<KeyPoint>& pt_set2, 
                        const Mat& K,
                        const Mat& Kinv,
                        const Mat& distcoeff,
                        const Matx34d& P,
                        const Matx34d& P1,
                        vector<CloudPoint>& pointcloud,
                        vector<KeyPoint>& correspImg1Pt)
{
#ifdef __SFM__DEBUG__
    vector<double> depths;
#endif
    
//  pointcloud.clear();
    correspImg1Pt.clear();
    
    Matx44d P1_(P1(0,0),P1(0,1),P1(0,2),P1(0,3),
                P1(1,0),P1(1,1),P1(1,2),P1(1,3),
                P1(2,0),P1(2,1),P1(2,2),P1(2,3),
                0,      0,      0,      1);
    Matx44d P1inv(P1_.inv());
    
    cout << "Triangulating...";
    double t = getTickCount();
    vector<double> reproj_error;
    unsigned int pts_size = pt_set1.size();
    Mat_<double> KP1 = K * Mat(P1);
//loop to traiangulate all the points from teh image

    for (int i=0; i<pts_size; i++) {
        Point2f kp = pt_set1[i].pt; 
        Point3d u(kp.x,kp.y,1.0);
        Mat_<double> um = Kinv * Mat_<double>(u); 
        u.x = um(0); u.y = um(1); u.z = um(2);

        Point2f kp1 = pt_set2[i].pt; 
        Point3d u1(kp1.x,kp1.y,1.0);
        Mat_<double> um1 = Kinv * Mat_<double>(u1); 
        u1.x = um1(0); u1.y = um1(1); u1.z = um1(2);
        
        Mat_<double> X = IterativeLinearLSTriangulation(u,P,u1,P1);
        
//      cout << "3D Point: " << X << endl;
//      Mat_<double> x = Mat(P1) * X;
//      cout << "P1 * Point: " << x << endl;
//      Mat_<double> xPt = (Mat_<double>(3,1) << x(0),x(1),x(2));
//      cout << "Point: " << xPt << endl;
        Mat_<double> xPt_img = KP1 * X;             //reproject
//      cout << "Point * K: " << xPt_img << endl;
        Point2f xPt_img_(xPt_img(0)/xPt_img(2),xPt_img(1)/xPt_img(2));
                
#pragma omp critical
        {
            double reprj_err = norm(xPt_img_-kp1);
            reproj_error.push_back(reprj_err);

            CloudPoint cp; 
            cp.pt = Point3d(X(0),X(1),X(2)); // Use this point to store all teh points in our defined 3d structure.
            cp.reprojection_error = reprj_err;
            pointcloud.push_back(cp);
            correspImg1Pt.push_back(pt_set1[i]);
#ifdef __SFM__DEBUG__
            depths.push_back(X(2));
#endif
        }
    }

    
    Scalar mse = mean(reproj_error);
    t = ((double)getTickCount() - t)/getTickFrequency();
    cout << "Done. ("<<pointcloud.size()<<"points, " << t <<"s, mean reproj err = " << mse[0] << ")"<< endl;
    
    //show "range image"
#ifdef __SFM__DEBUG__
    {
        double minVal,maxVal;
        minMaxLoc(depths, &minVal, &maxVal);
        Mat tmp(240,320,CV_8UC3,Scalar(0,0,0)); //cvtColor(img_1_orig, tmp, CV_BGR2HSV);
        for (unsigned int i=0; i<pointcloud.size(); i++) {
            double _d = MAX(MIN((pointcloud[i].z-minVal)/(maxVal-minVal),1.0),0.0);
            circle(tmp, correspImg1Pt[i].pt, 1, Scalar(255 * (1.0-(_d)),255,255), CV_FILLED);
        }
        cvtColor(tmp, tmp, CV_HSV2BGR);
        imshow("Depth Map", tmp);
        waitKey(0);
        destroyWindow("Depth Map");
    }   
#endif
    
    return mse[0];
}


/*****************************************************************************************************






*****************************************************************************************************/


point *Recoverpoint( vector<CloudPoint>& pointcloud  , point *p){   
for (unsigned int i = 0 ; i < pointcloud.size(); i++){
    p[i].P.x=pointcloud[i].pt.x;
    p[i].P.y=pointcloud[i].pt.y;
    p[i].P.z=pointcloud[i].pt.z;
}
return p;
} 
/***********************************************************************************************






*********************************************************************************************************/

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  cout<<"1 "<<endl ; 
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  cout<<"2 "<<endl ; 
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  cout<<"3 "<<endl ; 
  viewer->addCoordinateSystem (1.0);
  cout<<"4 "<<endl ; 
  viewer->initCameraParameters ();
  return (viewer);
}



/***********************************************************************************************




*********************************************************************************************************/
void visualization( pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
    std::cout<<"Pointcould size : "<<source_cloud->points.size()<<std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(source_cloud);
    //viewer->addCube (-colorImage.cols, 0, -colorImage.rows, 0, 0, timeLength);
    cout<<"Main user interaction waiting loop"<<endl;
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    viewer->close(); // kill viewer
    viewer.reset();
}


















/*****************************************************************************************************



******************************************************************************************************/
pcl::PointCloud<pcl::PointXYZ>::Ptr convertintopointcloud(point *p , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,int size){
    cout<<"Starting"<<endl;
    cout<<"size: "<<size<<endl;
    for (int i = 0 ; i <size ; i++){
        cout<<"In the loop "<<endl;
      pcl::PointXYZ point;
      point.x = p[i].P.x;
      point.y = p[i].P.y;
      point.z = p[i].P.z;
      cout<<" points   "<<point.x<<" , "<<point.y<<" , "<<point.z<<endl;
      cout<<"Please Help"<<endl;
      cloud->points.push_back(point);
    }
    cout<<"Please start"<<endl;
    return cloud ;
}
/*****************************************************************************************************

    
*********************************************************************************************************/


// pcl::PointCloud<pcl::PointXYZ>::Ptr transformpointcloud(coordinates transform,float theta , pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){

  
//   Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

//   // Define a translation of 2.5 meters on the x axis.
//   transform_2.translation() << transform.x, transform.y, transform.z;

//   // The same rotation matrix as before; tetha radians arround Z axis
//   transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));


//   // Executing the transformation
//   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//   // You can either apply transform_1 or transform_2; they are the same
//   transformPointCloud (*source_cloud, *transformed_cloud, transform_2);


//   return transformed_cloud ;


// }


// /********************************************************************************************************
// // function to calculate the change that is calculate the transformation . 
// *********************************************************************************************************/


void imshow_250x250(const string& name_, const Mat& patch) {
    Mat bigpatch; cv::resize(patch,bigpatch,Size(250,250));
    imshow(name_,bigpatch);
}



/***********************************************************************************



*************************************************************************************/
pcl::PolygonMesh pointcloudmesh( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveVTKFile ("mesh.vtk", triangles);

  return triangles;

 }




