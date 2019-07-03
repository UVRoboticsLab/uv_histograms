#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <uv_msgs/HistogramStamped.h>
#include <iostream>
#include <string>
#include <vector>

#define defaultFrame_ID "null_frame"
#define defaultHistogramTopic "/histogram"
#define defaultImageWidth 640
#define defaultImageHeight 480
using namespace std;

ros::Publisher histoImagePub,histoCameraInfoPub;
ros::Subscriber histoSubscriber;
sensor_msgs::Image histoImage;
sensor_msgs::CameraInfo histoCameraInfo;

string local_frame_id,histogramTopic;
double currentTime;
int width,height;

int initNodeParams()
{

}


int initImageInfoTopic()
{
  //  simCameraInfo.header.stamp=ros::Time::now();
  // simCameraInfo.header.frame_id=local_frame_id;
  // simCameraInfo.height=height;
  // simCameraInfo.width=width;
  //  simCameraInfo.distortion_model="plumb_bob";
  // simCameraInfo.D.resize(5);
  // simCameraInfo.D[0]=0.0;
  // simCameraInfo.D[1]=0.0;
  // simCameraInfo.D[2]=0.0;
  // simCameraInfo.D[3]=0.0;
  // simCameraInfo.D[4]=0.0;

  // simCameraInfo.K[0]=383.90521240234375;
  // simCameraInfo.K[1]=0.0;
  // simCameraInfo.K[2]=322.4174499511719;
  // simCameraInfo.K[3]=0.0;
  // simCameraInfo.K[4]=383.90521240234375;
  // simCameraInfo.K[5]=234.2659912109375;
  // simCameraInfo.K[6]=0.0;
  // simCameraInfo.K[7]=0.0;
  // simCameraInfo.K[8]=1.0;

  // simCameraInfo.R[0]=1.0;
  // simCameraInfo.R[1]=0.0;
  // simCameraInfo.R[2]=0.0;
  // simCameraInfo.R[3]=0.0;
  // simCameraInfo.R[4]=1.0;
  // simCameraInfo.R[5]=0.0;
  // simCameraInfo.R[6]=0.0;
  // simCameraInfo.R[7]=0.0;
  // simCameraInfo.R[8]=1.0;
    
  // simCameraInfo.P[0]=383.90521240234375;
  // simCameraInfo.P[1]=0.0;
  // simCameraInfo.P[2]=322.4174499511719;
  // simCameraInfo.P[3]=0.0;
  // simCameraInfo.P[4]=0.0;
  // simCameraInfo.P[5]=383.90521240234375;
  // simCameraInfo.P[6]=234.2659912109375;
  // simCameraInfo.P[7]=0.0;
  // simCameraInfo.P[8]=0.0;
  // simCameraInfo.P[9]=0.0;
  // simCameraInfo.P[10]=1.0;
  // simCameraInfo.P[11]=0.0;

  // simCameraInfo.binning_x=0;
  // simCameraInfo.binning_y=0;
  // simCameraInfo.roi.x_offset=0;
  // simCameraInfo.roi.y_offset=0;
  // simCameraInfo.roi.height=0;
  // simCameraInfo.roi.width=0;
  // simCameraInfo.roi.do_rectify=-1;

  return 0;
}

int initHistoImage(int width, int height)
{
  histoImage.header.frame_id=local_frame_id;
  histoImage.header.frame_id="base_link";
  histoImage.height=height;
  histoImage.width=width;
  histoImage.encoding="bgr8";
  histoImage.is_bigendian=0;
  histoImage.step=width*3;
  histoImage.data.resize(width*height*3);
}


int pubImage()
{
  currentTime = ros::Time::now().toSec();
  histoImage.header.stamp=ros::Time::now();
  histoImagePub.publish(histoImage);
  // simCameraInfo.header.stamp=ros::Time::now();
  // cameraInfoPub.publish(simCameraInfo);
}

/* HistogramStamped callback */

void convertHisto2Img(const uv_msgs::HistogramStamped& histo)
{
  int i,j,pxlPos,max=0;
  int binHeight;
  float imgHeight,binsSize;
  
  binsSize= width/256;
  imgHeight= height-20;

  /*erasing previous image*/
  for (i=0;i<width*height*3;i++){
      histoImage.data[i]=0;
    }
  
  if (!histo.is_normalized) {
    for (i=0;i<256;i++) 
      if (max<histo.data[i]) max=histo.data[i];
  } else max=1.0;
  /*computing line size */
  for (i=0;i<256;i++){
    binHeight=(int) imgHeight-(histo.data[i]/(float)max)*imgHeight+10;
 
    for(j=470;j>binHeight;j--) {     
      pxlPos=((int) j*width+i*3)*3;
      histoImage.data[pxlPos]=255;
      histoImage.data[pxlPos+1]=0;
      histoImage.data[pxlPos+2]=0;
    }

  }
  pubImage();
}


int getParams(ros::NodeHandle nh)
{
  int params=0;

  nh.getParam("uvHistogram2Image/histogram",histogramTopic);
  if (histogramTopic.size()==0) histogramTopic=defaultHistogramTopic;
  else params++;
  histoSubscriber = nh.subscribe(histogramTopic, 1, convertHisto2Img);

  width=defaultImageWidth;
  height=defaultImageHeight;
  // nh.getParam("depth2pcd/depth_camera_info",cameraInfoTopic);
  // if (cameraInfoTopic.size()==0)  cameraInfoTopic=defaultCameraInfoTopic;
  // else params++;
  // subInfo = nh.subscribe(cameraInfoTopic, 1, setCameraInfo);

  initHistoImage(width, height);
  return params;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uvHistogram2Image");  
  ros::NodeHandle _nh;

  getParams(_nh);

  histoImagePub = _nh.advertise<sensor_msgs::Image>("/histogram_image",10);
  //  cameraInfoPub = _nh.advertise<sensor_msgs::CameraInfo>("/simcamera/depth/camera_info",10);
  
  ros::Rate loop_rate(25);
  //  initImageInfoTopic();
      
  while (ros::ok()) { 
    ros::spinOnce();
    loop_rate.sleep();    //pubDepthImage();
  }
  return 0;
}
