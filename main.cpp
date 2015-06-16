#include <math.h>
#include <stdio.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "blob.h"
#include "BlobContour.h"
#include "BlobLibraryConfiguration.h"
#include "BlobOperators.h"
#include "BlobProperties.h"
#include "BlobResult.h"
#include "ComponentLabeling.h"

#include "SerialClass.h"	


#define PI 3.14159265
#define MAX_BLUR_KERNEL_LENGTH  (19)

using namespace std;
using namespace cv;

void robot_mouse_callback( int event, int x, int y, int flags, void* param );

int robot_min_x=0, robot_min_y=0, robot_max_x=0, robot_max_y=0;
int robot_center_x=0,robot_center_y=0;
float robot_angle=0.0;
bool is_destination_availible=false;
int destination_x=0, destination_y=0;
float destination_angle=0.0;


Mat redFilter(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat red_only;
    inRange(src, Scalar(0, 0, 50), Scalar(35, 30, 160), red_only);

    return red_only;
}

Mat blueFilter(const Mat& src)
{
    assert(src.type() == CV_8UC3);

    Mat blue_only;
    inRange(src, Scalar(70, 35, 15), Scalar(160, 90, 60), blue_only);

    return blue_only;
}


int main(int argc, char** argv)
{

	Serial* SP = new Serial("\\\\.\\COM18");    // adjust as needed
	if (SP->IsConnected())
		printf("We're connected to serial controller\r\n");
	else
	{
		printf("Serial connect error\r\n");
		return -1;
	}
	SP->WriteData("Stop!",strlen("Stop!"));



	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

	int erosion_size = 1;
	Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ) );

	Mat frame;
	cap >> frame;
	if(frame.data == NULL)
	{
			printf("Frame is NULL!\r\n");
			return -1;
	}

	while(1)
	{
		cap >> frame;

		if(frame.data == NULL)
		{
				printf("Frame is NULL!\r\n");
				return -1;
		}

		cvSetMouseCallback("frame", robot_mouse_callback, (void*)&frame);

		
		for (int i = 1; i < MAX_BLUR_KERNEL_LENGTH; i = i + 2 )
		{ 
			GaussianBlur( frame, frame, Size( i, i ), 0, 0 );
		}  

		Mat blueImg = blueFilter(frame);
		erode(blueImg,blueImg,element);
		dilate( blueImg, blueImg, element );


		Mat redImg = redFilter(frame);
		erode(redImg,redImg,element);
		dilate( redImg, redImg, element );

		CBlobResult blobs;
		IplImage* img = new IplImage(redImg);
		blobs = CBlobResult( img, NULL, 0 );
		blobs.Filter( blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 200 );

		int blobNums =  blobs.GetNumBlobs();
		int rx=0,ry=0;
		CvRect redBlobBoundingBox;
		if(blobNums)
		{

		CBlob currentBlob = blobs.GetBlob(0);
		redBlobBoundingBox = currentBlob.GetBoundingBox();
		rx = currentBlob.MinX() + (redBlobBoundingBox.width/2);
		ry = currentBlob.MinY() + (redBlobBoundingBox.height/2);
		}
		

		img = new IplImage(blueImg);
		blobs = CBlobResult( img, NULL, 0 );
		blobs.Filter( blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 200 );

		int bx=0, by=0;
		CvRect blueBlobBoundingBox;
		if(blobs.GetNumBlobs())
		{
			CBlob currentBlob = blobs.GetBlob(0);
			blueBlobBoundingBox = currentBlob.GetBoundingBox();
			bx = currentBlob.MinX() + (blueBlobBoundingBox.width/2);
			by = currentBlob.MinY() + (blueBlobBoundingBox.height/2);
		}

		line(frame,Point(rx,ry),Point(bx,by),Scalar(0,0,255));
		robot_angle = -atan2(ry - by , rx - bx ) * 180 / PI;

		Mat robot;
		add(redImg,blueImg,robot);
		img = new IplImage(robot);
		blobs = CBlobResult( img, NULL, 0 );
		blobs.Filter( blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, 200 );

		robot_min_x = 0, robot_min_y=0, robot_max_x=0, robot_max_y=0;
		for(int i=0; i< blobs.GetNumBlobs(); i++)
		{
			CBlob robot_Blob = blobs.GetBlob(i);
			if(robot_min_x == 0 || robot_min_x > robot_Blob.MinX())
			{
				robot_min_x = robot_Blob.MinX();
			}

			if(robot_min_y == 0 || robot_min_y > robot_Blob.MinY())
			{
				robot_min_y = robot_Blob.MinY();
			}

			if(robot_max_x == 0 || robot_max_x < robot_Blob.MaxX())
			{
				robot_max_x = robot_Blob.MaxX();
			}

			if(robot_max_y == 0 || robot_max_y < robot_Blob.MaxY())
			{
				robot_max_y = robot_Blob.MaxY();
			}
	
		};

		robot_center_x = (robot_min_x + robot_max_x)/2;
		robot_center_y = (robot_min_y + robot_max_y)/2;
		rectangle(frame,Point(robot_min_x,robot_min_y),Point(robot_max_x,robot_max_y),Scalar(0,255,0));

		if(is_destination_availible)
		{
			line(frame, Point(robot_center_x,robot_center_y),Point(destination_x,destination_y),Scalar(255,0,0));
			bool is_rotation_required=true;
			if(destination_angle>robot_angle)
			{
				float diff = destination_angle - robot_angle;
				if(diff > 10.0)
				{
					SP->WriteData("Left!",strlen("Left!"));
					is_rotation_required = true;
				}
				else
				{
					is_rotation_required = false;
				}
			}
			else
			{
				float diff =   robot_angle - destination_angle;
				if(diff > 10.0)
				{
					SP->WriteData("Right!",strlen("Right!"));
					is_rotation_required = true;
				}
				else
				{
					is_rotation_required = false;
				}
			}

			if(is_rotation_required == false)
			{
				int x_diff = (robot_center_x - destination_x);
				int y_diff = (robot_center_y - destination_y);
				int distance = sqrt( (x_diff*x_diff) + (y_diff*y_diff) );

				if(distance > 50)
				{
					SP->WriteData("Go!",strlen("Go!"));
				}
				else
				{
					SP->WriteData("Stop!",strlen("Stop!"));
					is_destination_availible = false;
				}
			}

		}

		

		imshow("frame",frame);
		if(waitKey(60) >= 0) break;
	}
	SP->WriteData("Stop!",strlen("Stop!"));
	

    return 0;
}
 


void robot_mouse_callback( int event, int x, int y, int flags, void* param ){
	Mat* image = (Mat*) param;

	switch( event ){
		case CV_EVENT_MOUSEMOVE: 
			break;

		case CV_EVENT_LBUTTONDOWN:
			printf("Clicked to x=%d, y=%d\r\n",x,y);
			printf("Robot angle = %f degree\r\n", robot_angle);
			is_destination_availible = true;
			destination_x = x;
			destination_y = y;
			destination_angle = -atan2(destination_y - robot_center_y , destination_x - robot_center_x ) * 180 / PI;
			printf("Destination angle = %f degree\r\n", destination_angle);
			break;

		case CV_EVENT_LBUTTONUP:

			break;
	}
}
