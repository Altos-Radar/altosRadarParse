#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include"pointCloud.h"
#include<math.h>
#include <sys/time.h>
#include <algorithm>
using namespace std;
#define widthSet 4220
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
float hist(float *vr,float *histBuf,float step,int vrInd)
{
    int ind = 0;
    for(int i = 0;i<vrInd;i++)
    {
        ind = (vr[i] - vrMin)/step;
        if(vr[i]>60||vr[i]<-60)
        {
            // printf("vr[%d] = %f\n",ind,vr[i]);
            continue;
        }

        histBuf[ind]++;
    }
    return float((max_element(histBuf,histBuf+(int((vrMax-vrMin)/step))) - histBuf))*step+vrMin;
}
int main(int argc,char **argv)
{
    
    struct sockaddr_in addr;
    struct sockaddr_in from;
    struct ip_mreq req;
    socklen_t len = sizeof(from);
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd)
    {
        perror("socket"); 
        return 0;
    }
    struct timeval timeout = {1,300}; 
    setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(struct timeval));
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(addr));
    if (-1 == ret)
    {
        perror("bind"); 
        return 0;
    }
    req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
    req.imr_interface.s_addr = inet_addr(/*"0.0.0.0"*/"192.168.3.1");;
    ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    if (ret < 0)
    {
        perror("setsockopt"); 
        return 0;
    }
    
    int recvFrameLen = 0;
	int frameNum = 0;
    int tmp;
    POINTCLOUD pointCloudBuf;
	char* recvBuf= (char*)&pointCloudBuf;
    POINT *points = (POINT*)malloc(sizeof(POINT)*widthSet);
    memset(points,0,sizeof(POINT)*widthSet);

    int frameId = 0;
    int objectCntFrame = 0;
    int i;
    int objectCntLast,objectCnt;
    float vr[widthSet];
    float vrAzi[widthSet];
    float step = 0.2;
    float *histBuf = (float*)malloc(sizeof(float)*int((vrMax-vrMin)/step));
    int vrInd = 0;
    float vrEst = 0;
    int cntFrameobj = 30;

    while(1)
    {
        usleep(1);
        ret = recvfrom(sockfd, recvBuf, 1368, 0, (struct sockaddr *)&from, &len);
        if (ret > 0)
		{
            pointCloudBuf.pckHeader.curObjNum = pointCloudBuf.pckHeader.curObjNum/44;
            objectCnt = pointCloudBuf.pckHeader.objectCount;
            pointCloudBuf.pckHeader.curObjInd = pointCloudBuf.pckHeader.curObjInd*30;
            tmp = pointCloudBuf.pckHeader.frameId;
            if(frameId == 0 || frameId == tmp)
            {
                frameId = tmp;
                for(i = 0;i<pointCloudBuf.pckHeader.curObjNum;i=i+1)
                {
                    if(abs(pointCloudBuf.point[i].range)>0)
                    {
                        pointCloudBuf.point[i].ele = -(pointCloudBuf.point[i].ele);
                        pointCloudBuf.point[i].azi = asin(sin(pointCloudBuf.point[i].azi)/cos(pointCloudBuf.point[i].ele));
                        points[pointCloudBuf.pckHeader.curObjInd+i].x = (pointCloudBuf.point[i].range)*cos(pointCloudBuf.point[i].azi)*cos(pointCloudBuf.point[i].ele); 
                        points[pointCloudBuf.pckHeader.curObjInd+i].y = (pointCloudBuf.point[i].range)*sin(pointCloudBuf.point[i].azi)*cos(pointCloudBuf.point[i].ele);; 
                        points[pointCloudBuf.pckHeader.curObjInd+i].z = (pointCloudBuf.point[i].range)*sin(pointCloudBuf.point[i].ele) ; 
                        points[pointCloudBuf.pckHeader.curObjInd+i].h = pointCloudBuf.point[i].doppler; 
                        points[pointCloudBuf.pckHeader.curObjInd+i].s = 10*log10(pointCloudBuf.point[i].snr); 
                        vr[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].doppler/cos(pointCloudBuf.point[i].azi);
                        vrAzi[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].azi;
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                objectCntFrame = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                objectCntLast = objectCnt;
                cntFrameobj = cntFrameobj + 30;

            }else{
                if(cntFrameobj<objectCntLast)
                {
                    printf("-------------------------dataLoss %d\t%d\t%d pack(s) in %d packs------------------------\n",cntFrameobj,objectCntLast,(objectCntLast - cntFrameobj)/POINTNUM,objectCntLast/POINTNUM);
                }
                memset(histBuf,0,sizeof(float)*int((vrMax-vrMin)/step));
                vrEst = hist(vr,histBuf,step,vrInd);
                printf("Frame %d: objectCnt is %d\n",frameId,objectCntLast);
                cntFrameobj = POINTNUM;
                objectCntLast = objectCnt;
                for(i = 0;i<vrInd;i++)
                {
                    points[i].v = points[i].h - vrEst*cos(vrAzi[i]);
                    if(points[i].v<-errThr)
                    {
                        points[i].v = -1;
                    }else if(points[i].v>errThr)
                    {
                        points[i].v = 1;
                    }else{
                        points[i].v = 0;
                    }
                }
                memset(points,0,sizeof(POINT)*widthSet);
                frameId = tmp;
                for(int i = 0;i<pointCloudBuf.pckHeader.curObjNum;i++)
                {
                    if(abs(pointCloudBuf.point[i].range)>0)
                    {
                        pointCloudBuf.point[i].ele = -(pointCloudBuf.point[i].ele);
                        pointCloudBuf.point[i].azi = asin(sin(pointCloudBuf.point[i].azi)/cos(pointCloudBuf.point[i].ele));
                        points[pointCloudBuf.pckHeader.curObjInd+i].x = (pointCloudBuf.point[i].range)*cos(pointCloudBuf.point[i].azi); 
                        points[pointCloudBuf.pckHeader.curObjInd+i].y = (pointCloudBuf.point[i].range)*sin(pointCloudBuf.point[i].azi);
                        points[pointCloudBuf.pckHeader.curObjInd+i].z = (pointCloudBuf.point[i].range)*sin(pointCloudBuf.point[i].ele) ; 
                        points[pointCloudBuf.pckHeader.curObjInd+i].h = pointCloudBuf.point[i].doppler; 
                        points[pointCloudBuf.pckHeader.curObjInd+i].s = 10*log10(pointCloudBuf.point[i].snr); 
                        vr[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].doppler/cos(pointCloudBuf.point[i].azi);
                        vrAzi[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].azi;    
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
            }
		}else
        {
            printf("recv failed (timeOut)   %d\n",ret);
        }
    }
    close(sockfd);
    free(histBuf);
    free(points);
    return 0;
}
