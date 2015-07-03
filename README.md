# K-means-for-cluster-formation
WSN
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "basestation.h"
#include "sensor.h"
#include <cstddev.h>
#include<math.h>

Define_Module(BaseStation);

void BaseStation::initialize()
{//int z=1;
    // Arrange sensors
    cModule *wsn = simulation.getModuleByPath("Wsn");
    double wsnWidth = wsn->par("width");
    double wsnHeight = wsn->par("height");
    int ssRows = wsn->par("ssRows");
    int ssCols = wsn->par("ssCols");
    int numSensors = wsn->par("numSensors");
    cluster=wsn->par("cluster");
    char modPath[100]; // Module path for sensors
    Sensor *ss;

    this->setX(this->par("x"));
    this->setY(this->par("y"));

    int i = 0,index;
     n = numSensors <= ssRows * ssCols ? numSensors : ssRows * ssCols; // Number of sensors which will be arranged
    numSensors=n;
    for (i = 0; i < n; i++) {
        sprintf(modPath, "Wsn.sensor[%d]", i); // Create module path string
        ss = check_and_cast<Sensor*>(getModuleByPath(modPath));
        ss->setX((i % ssCols) * rint(wsnWidth / ssCols) + intuniform(0, wsnWidth / ssCols / 2));
        ss->setY((i / ssCols) * rint(wsnHeight / ssRows) + intuniform(0, wsnHeight / ssRows / 2));
        ss->n=this->n;

    }
    //this->buildCluster(cluster,n);
     //EV << "Roundhfdddffffffffffffffffffffffffffffffffffffffffffffff" <<z<<endl;
     //z++;
    scheduleAt(0, new cMessage);

}


void BaseStation::handleMessage(cMessage *msg)
{
    if(msg->isSelfMessage())
      { if(n==48) {
          int temp[4]={10,13,28,38};
          this->makeCluster(temp,4);
      }
      else
        this->buildCluster(cluster,n);

delete msg;
   }


      }
  //  scheduleAt(simTime()+0.1,new cMessage);
    // TODO - Generated method body
void BaseStation::mySort(int *a,int k)
{
    int i,j,temp;
    for(i=0;i<k-1;i++)
    {
        for(j=i+1;j<k;j++)
        {
            if(a[i]>a[j])
            {
                temp=a[i];
                a[i]=a[j];
                a[j]=temp;
            }
        }
    }
}
bool BaseStation::isSame(int *temp1,int *temp2,int k)
{
    int i;
    mySort(temp1,k);
    mySort(temp2,k);
    for(i=0;i<k;i++)
    {
        if(temp1[i]!=temp2[i])
            return false;
    }
    return true;
}
void BaseStation::distSort(double *distance,int *node,int k)
{
    int i,j,temp;
    double temp1;
    Sensor *ss1[48];
    char modPath[20];
    for(i=0;i<n;i++)
          {

              sprintf(modPath, "Wsn.sensor[%d]", i); // Create module path string
                      ss1[i]= check_and_cast<Sensor*>(getModuleByPath(modPath));
          }
       for(i=0;i<k-1;i++)
       {
           for(j=i+1;j<k;j++)
           {
               if(distance[i]>distance[j])
               {
                   temp=node[i];
                   node[i]=node[j];
                   node[j]=temp;
                   temp1=distance[i];
                   distance[i]=distance[j];
                   distance[j]=temp1;

               }
           }
           ss1[node[i]]->index=i;
           EV<<node[i]<<"   "<<i<<endl;
       }
   }

void BaseStation::makeCluster(int *temp,int k)
{
    int i,ss[48],j=0,index,m=4;
    Sensor *ss1[48];
    char modPath[20];
    int in1[48],out[48];
    double temp1,min,dist;
    cDatarateChannel *channel = NULL;
    int sensenum[48];

    for(i=0;i<n;i++)
           {

          sprintf(modPath, "Wsn.sensor[%d]", i); // Create module path string
          ss1[i]= check_and_cast<Sensor*>(getModuleByPath(modPath));
          in1[i]=0;
          out[i]=0;
          sensenum[i]=0;
          ss1[i]->num=0;
           }
    temp1=ss1[n-1]->distance(ss1[0]);
        for(i=0;i<n;i++)
           {   j=3;
           min=temp1;
               while(j>=0)
               {
                   dist=ss1[i]->distance(ss1[temp[j]]);
                   if(dist<min)
                   {
                       min=dist;
                       index=temp[j];
                   }
                   j--;
               }

    ss[i]=index;
    sensenum[ss[i]]++;
    ss1[i]->num++;
}
        int bsgate=0;
        channel = cDatarateChannel::create("channel");
                channel->setDelay(0.01);
                channel->setBitErrorRate(0);
                channel->setDatarate(1e6);
                ss1[temp[0]]->gate("gate$o",out[temp[0]]++)->connectTo(this->gate("in",bsgate++),channel);
                ss1[temp[0]]->clusterhead=true;
                ss1[temp[0]]->updateDisplay();
                ss1[temp[0]]->destaddr=temp[0];
                ss1[temp[0]]->clusterHeadId=ss1[temp[0]]->getId();
                ss1[temp[0]]->outGoingId= this->getId();
              //  ss1[temp[0]]->cluster=(int *)malloc(sensenum[temp[0]]*sizeof(int));
                ss1[temp[0]]->baseIndex=bsgate-1;
                ss1[temp[0]]->num=sensenum[temp[0]];

                ss1[temp[0]]->cluster[0]=temp[0];
                ss1[temp[0]]->distanceVector[0]=0;


                for(j=1;j<4;j++)
                {
                    channel = cDatarateChannel::create("channel");
                           channel->setDelay(0.01);
                           channel->setBitErrorRate(0);
                           channel->setDatarate(1e6);


                    ss1[temp[j]]->gate("gate$o",out[temp[j]]++)->connectTo(this->gate("in",bsgate++),channel);
                    ss1[temp[j]]->clusterhead=true;
                    ss1[temp[j]]->updateDisplay();
                   // ss1[temp[j]]->destaddr=temp[j-1];
                    ss1[temp[j]]->clusterHeadId=ss1[temp[j]]->getId();
                    ss1[temp[j]]->outGoingId=this->getId();
                   // ss1[temp[j-1]]->incomingId=ss1[temp[j]]->getId();
                  //  ss1[temp[j]]->cluster=(int *)malloc(sensenum[temp[j]]*sizeof(int));
                    ss1[temp[j]]->cluster[0]=temp[j];
                    ss1[temp[j]]->baseIndex=bsgate-1;
                    ss1[temp[j]]->num=sensenum[temp[j]];
                    ss1[temp[j]]->distanceVector[0]=0;





                }
            temp1=ss1[n-1]->distance(ss1[0]);
                for(i=0;i<n;i++)
                   {   j=m-1;
                   min=temp1;
                       while(j>=0)
                       {
                           dist=ss1[i]->distance(ss1[temp[j]]);
                           if(dist<min)
                           {
                               min=dist;
                               index=temp[j];
                           }
                           j--;
                       }

                             channel = cDatarateChannel::create("channel");
                                    channel->setDelay(0.01);
                                    channel->setBitErrorRate(0);
                                    channel->setDatarate(1e6);

                                    if(i!=index)
                                    {
                       ss1[i]->gate("gate$o",out[i]++)->connectTo(ss1[index]->gate("gate$i",in1[index]++),channel);
                         ss1[i]->destaddr=index;
                          ss1[i]->clusterHeadId=ss1[index]->getId();
                          ss1[i]->outGoingId=ss1[index]->getId();
                          ss1[index]->cluster[in1[index]]=i;
                          ss1[index]->distanceVector[in1[index]]=ss1[i]->distance(ss1[index]);
                                    }
                   }
                for(i=0;i<4;i++)
                    distSort(ss1[temp[i]]->distanceVector,ss1[temp[i]]->cluster,sensenum[temp[i]]);

}


void BaseStation::buildCluster(int k,int n)
{//cModule *wsn = simulation.getModuleByPath("Wsn");

    Sensor *ss1[100];
    int temp[50],temp2[50];
    int ss[100],sensenum[50],x[50],y[50],x1,y1;
    int i=0,index,m=0,j;
    char modPath[100];
    double dist,min,temp1;


    // Module path for sensors
     while(i<n)
     {
        temp[m]=i+intrand((int)n/k);
         m++;
         i=i+(int)n/k;

        // temp[i++]=intrand(n);

     }

    for(i=0;i<n;i++)
       {

           sprintf(modPath, "Wsn.sensor[%d]", i); // Create module path string
                   ss1[i]= check_and_cast<Sensor*>(getModuleByPath(modPath));
      sensenum[i]=0;
      x[i]=0;
      y[i]=0;

       }
    do{
        EV<<"loop"<<endl;
        for(i=0;i<k;i++)
        {
            temp2[i]=temp[i];
            x[temp[i]]=0;
            y[temp[i]]=0;
            sensenum[temp[i]]=0;
        }
    temp1=ss1[0]->distance(ss1[n-1]);
    for(i=0;i<n;i++)
    {min=temp1;
    for(j=0;j<m;j++)
    {
        dist=ss1[i]->distance(ss1[temp[j]]);
        if(min>dist)
        {
            min=dist;
            index=temp[j];
        }
    }
    ss[i]=index;
    x[index]+=ss1[i]->getX();
    y[index]+=ss1[i]->getY();
    sensenum[index]++;
    }
    for(j=0;j<m;j++)
    {
        x1=(int)(x[temp[j]]/sensenum[temp[j]]);

        y1=(int)(y[temp[j]]/sensenum[temp[j]]);
        min=temp1;
        for(i=0;i<n;i++)
        {
            dist=sqrt((x1-ss1[i]->getX())*(x1-ss1[i]->getX())+(y1-ss1[i]->getY())*(y1-ss1[i]->getY()));
            if(min>dist)
            {
                min=dist;
                temp[j]=i;
            }
        }

    }
    }while(!isSame(temp,temp2,k));
    for(i=0;i<k;i++)
        EV<<temp[i]<<endl;

     this->makeCluster(temp,k);


}




