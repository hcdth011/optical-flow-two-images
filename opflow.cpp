#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

const int MAX_CORNERS = 500;

int main(int argc, char* argv[]) { 
  // Load two images and allocate other structures
  IplImage* imgA = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  IplImage* imgB = cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

  CvSize img_sz = cvGetSize(imgA);
  int win_size = 15;

  IplImage* imgC = cvLoadImage(argv[1], CV_LOAD_IMAGE_UNCHANGED);

  // Get the features for tracking
  IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
  IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

  int corner_count = MAX_CORNERS;
  CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

  cvGoodFeaturesToTrack( imgA, eig_image, tmp_image, cornersA, &corner_count,
    0.05, 5.0, 0, 3, 0, 0.04 );

  cvFindCornerSubPix( imgA, cornersA, corner_count, cvSize( win_size, win_size ),
    cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

  // Call Lucas Kanade algorithm
  char features_found[ MAX_CORNERS ];
  float feature_errors[ MAX_CORNERS ];

  CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );

  IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
  IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

  CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];

  cvCalcOpticalFlowPyrLK( imgA, imgB, pyrA, pyrB, cornersA, cornersB, corner_count, 
    cvSize( win_size, win_size ), 5, features_found, feature_errors,
     cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

  // Make an image of the results

  for( int i=0; i<550; i++ ){
    printf("Error is %f\n", feature_errors[i]);
    if(feature_errors[i]<0.001) continue;

    printf("Got it\n");
    CvPoint p0 = cvPoint( cvRound( cornersA[i].x ), cvRound( cornersA[i].y ) );
    CvPoint p1 = cvPoint( cvRound( cornersB[i].x ), cvRound( cornersB[i].y ) );
    cvLine( imgC, p0, p1, CV_RGB(255,0,0), 2 );
  }


  cvNamedWindow( "ImageA", 0 );
  cvNamedWindow( "ImageB", 0 );
  cvNamedWindow( "LKpyr_OpticalFlow", 0 );

  cvShowImage( "ImageA", imgA );
  cvShowImage( "ImageB", imgB );
  cvShowImage( "LKpyr_OpticalFlow", imgC );

  cvWaitKey(0);


  return 0;
}
