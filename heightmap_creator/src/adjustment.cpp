#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src, src2; Mat src_gray;
int thresh = 8;
int max_thresh = 255;
RNG rng(12345);
Mat warp_mat( 2, 3, CV_32FC1 );
/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );
  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  thresh_callback(0, 0);


  Mat warp_dst, warp_dst2;
  warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
  warpAffine( src2, warp_dst2, warp_mat, warp_dst2.size() );
  char* warp_window = "Warped";
  cout << warp_dst.size() << endl;
  namedWindow( warp_window, CV_WINDOW_AUTOSIZE );
  imshow( warp_window, warp_dst );

  imwrite("heightmapcolor.jpg", warp_dst);
  imwrite("heightmapimg.jpg", warp_dst2);
  waitKey(0);
  return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<RotatedRect> boundRect( contours.size() );
  vector<float>radius( contours.size() );
  Point2f Apriltagbox[4];
  for( int i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = minAreaRect( Mat(contours_poly[i]) );
     }


  /// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       float size = boundRect[i].size.width*boundRect[i].size.height;
       //cout << size << endl;
       if (size>15000 && size<18000){
       	for( int j = 0; j < 4; j++ ){
       	  boundRect[i].points(Apriltagbox);
          line( drawing, Apriltagbox[j], Apriltagbox[(j+1)%4], Scalar(0,255,0), 1, 8 );  
          //cout << Apriltagbox[j] << endl;
       	}

       }
 
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  //transform to groundtruth
  //top_left: 160-65=95 112-65=47 130*130
  Point2f srcpnts[4];
  Point2f dstpnts[4];

  srcpnts[0] = Apriltagbox[1];
  srcpnts[1] = Apriltagbox[0];
  srcpnts[2] = Apriltagbox[3];
  srcpnts[3] = Apriltagbox[2];
  dstpnts[0] = Point2f(95, 47);
  dstpnts[1] = Point2f(95, 47+130);
  dstpnts[2] = Point2f(95+130, 47+130);
  dstpnts[3] = Point2f(95+130, 47);
  cout << Apriltagbox[0] << Apriltagbox[1] << Apriltagbox[2] << Apriltagbox[3] << endl;
  
  warp_mat = getAffineTransform( srcpnts, dstpnts );


}
