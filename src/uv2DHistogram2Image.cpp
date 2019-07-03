#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <uv_msgs/Histogram2DStamped.h>
#include <iostream>
#include <string>
#include <vector>

#define defaultFrame_ID "null_frame"
#define defaultHistogramTopic "/2Dhistogram"
#define defaultImageWidth 500
#define defaultImageHeight 350
using namespace std;

ros::Publisher histo2DImagePub,histoCameraInfoPub;
ros::Subscriber histoSubscriber;
sensor_msgs::Image histoImage;
sensor_msgs::CameraInfo histoCameraInfo;
uv_msgs::Histogram2DStamped th2DHisto;

string local_frame_id,histogramTopic;
double currentTime;
int width,height;

int initNodeParams()
{

}

int histo2DThresholding(int Th,const float * hData)
{
  int size=th2DHisto.xBins*th2DHisto.yBins;
 
  for (int i=0;i<size;i++) {
    if (hData[i]>Th) th2DHisto.data[i]=1.0;
    else th2DHisto.data[i]=0.0;
  }

  
  return 0;
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
  
  th2DHisto.is_normalized=false;
  th2DHisto.xBins=500;
  th2DHisto.yBins=350;
  th2DHisto.data.resize(500*350);
  
}


int pubImage()
{
  currentTime = ros::Time::now().toSec();
  histoImage.header.stamp=ros::Time::now();
  histo2DImagePub.publish(histoImage);
  // simCameraInfo.header.stamp=ros::Time::now();
  // cameraInfoPub.publish(simCameraInfo);
}

/* HistogramStamped callback */

void convertHisto2DToImg(const uv_msgs::Histogram2DStamped& histo)
{
  int i,j,pxlPos,max=0;
  int binHeight;
  float imgHeight,binsSize;
  const float *hData;
  
  binsSize= width/256;
  imgHeight= height-20;

  hData=&histo.data[0];
  
  /*erasing previous image*/
  for (i=0;i<width*height*3;i++){
      histoImage.data[i]=0;
    }

  if (!histo.is_normalized) {
    for (i=0;i<width*height;i++) {
      if (max<histo.data[i]) 
	max=histo.data[i];
    }
    
    for (i=0;i<width*height;i++){
    //  binHeight=(int) (( histo.data[i]/max)*255.0);
    binHeight=(int) (( histo.data[i]/histo.max_value)*255.0);
    histoImage.data[i*3]=binHeight;
    histoImage.data[i*3+1]=binHeight;
    histoImage.data[i*3+2]=binHeight;
    // th2DHisto.is_normalized=true;
    // th2DHisto.max_value=1.0;
    }

  } else {
    //    cout << "max_value " << histo.max_value << '\n';
  
    //    histo2DThresholding(0.1, hData);

    for (i=0;i<width*height;i++){
      //  binHeight=(int) (( histo.data[i]/max)*255.0);
      if (histo.data[i]>0.1) {
	binHeight= 255;//(int) (histo.data[i]*255.0);
      } else binHeight=0;
	histoImage.data[i*3]=binHeight;
	histoImage.data[i*3+1]=binHeight;
	histoImage.data[i*3+2]=binHeight;
      
    }
  }
  pubImage();
}


int getParams(ros::NodeHandle nh)
{
  int params=0;

  nh.getParam("uv2DHistogram2Image/histogram",histogramTopic);
  if (histogramTopic.size()==0) histogramTopic=defaultHistogramTopic;
  else params++;
  histoSubscriber = nh.subscribe(histogramTopic, 1, convertHisto2DToImg);

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
  ros::init(argc, argv, "uv2DHistogram2Image");  
  ros::NodeHandle _nh;

  getParams(_nh);

  histo2DImagePub = _nh.advertise<sensor_msgs::Image>("/2Dhistogram_image",10);
  //  cameraInfoPub = _nh.advertise<sensor_msgs::CameraInfo>("/simcamera/depth/camera_info",10);
  
  ros::Rate loop_rate(25);
  //  initImageInfoTopic();
      
  while (ros::ok()) { 
    ros::spinOnce();
    loop_rate.sleep();    
  }
  return 0;
}
