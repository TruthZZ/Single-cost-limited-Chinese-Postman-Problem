import sys;
import numpy as np;
import os;
import csv;
import math;

class road:
    def __init__(self,node1,node2,length,roadNO):
        super().__init__();
        self.initR(node1,node2,length,roadNO);

    def initR(self,node1,node2,inlength,roadNO):
        self.RNO = roadNO;
        self.nodes = [node1,node2];
        self.length = inlength;
        self.times = 0;
        self.firstdis = 0;
        self.rootnodes = [];
        self.diffdis = 0;

        #self.distances = np.zeros(3,74);

    def findmindis(self,otherroads,nodescoor):
        self.nodeorder = np.zeros([74,2]);
        self.minmindis = [];
        num = 0;
        for road in otherroads:
            if road.RNO != self.RNO:
                templen = road.length;
                disaa = self.caldis(self.nodes[0],road.nodes[0],nodescoor);
                disab = self.caldis(self.nodes[0],road.nodes[1],nodescoor);
                disba = self.caldis(self.nodes[1],road.nodes[0],nodescoor);
                disbb = self.caldis(self.nodes[1],road.nodes[1],nodescoor);

                min1 = min(disaa,disba);
                min2 = min(disab,disbb);

                if min1 == disaa:
                    node1 = self.nodes[0];
                elif min1 == disba:
                    node1 = self.nodes[1];

                if min2 == disab:
                    node2 = self.nodes[0];
                elif min2 == disbb:
                    node2 = self.nodes[1];

                self.nodeorder[num,0] = node1;
                self.nodeorder[num,1] = node2;

                num = num + 1;

                mindis = templen + min1 + min2;
                self.minmindis.append(mindis);


    def caldis(self,node1,node2,nodescoor):
        nodes = nodescoor;
        r = node1-1;
        l = node2-1;

        xlat = nodes[r,0];
        xlon = nodes[r,1];
        xlat_rad = xlat*math.pi/180;
        xlon_rad = xlon*math.pi/180;

        ylat = nodes[l,0];
        ylon = nodes[l,1];
        ylat_rad = ylat*math.pi/180;
        ylon_rad = ylon*math.pi/180;

        dlat = abs(xlat-ylat);
        dlon = abs(xlon-ylon);

        dlat_rad = dlat*math.pi/180;
        dlon_rad = dlon*math.pi/180;

        s = 2*math.asin(math.sqrt(math.pow(math.sin(dlat_rad/2),2)+math.cos(xlat_rad)*math.cos(ylat_rad)*math.pow(math.sin(dlon_rad/2),2)));
        distance = s * 6378137/1000;
        
        return distance;



class Findroutes:
    def __init__(self,baseA,baseB,baseC,roads,nodescoor):
        super().__init__();
        self.initFR(baseA,baseB,baseC,roads,nodescoor);

    def initFR(self,baseA,baseB,baseC,roads,nodescoor):
        self.roads = roads;
        self.maxflight = 50;
        self.nodelist = [];
        self.nodestack = [];
        self.waitlist = roads;
        self.restflt = self.maxflight;
        self.waitlen = len(self.waitlist);
        self.failure = 0;
        self.numlist = [];
        self.endslist = [];
        self.resultlist = [];
        self.calfirstdis(baseA,baseB,baseC,roads,nodescoor);
        print('Firstdis:'+str(self.firstdis));
        print('num<50:'+str(sum(self.firstdis[x]<=50 for x in range(len(self.firstdis)))));

        for i in range(1,76):
            self.numlist.append(i);
            self.endslist.append(self.roads[i-1].nodes);

        #print(self.endslist);

        self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);

        self.iii = 0;
        while self.iii < len(self.waitlist):
            road = self.decendroads[self.iii];
            self.restflt = self.maxflight;

            #if len(self.nodestack) != 0:
             #   self.nodelist.append(self.nodestack.pop());
              #  self.nodelist.append(self.nodestack.pop());
            self.resultlist.append(self.nodelist);
            self.nodelist = [];

            print('restflt:'+str(self.restflt)+'firstdis:'+str(road.firstdis));

            if road.firstdis <= self.restflt:
                #print('Yes!');
                self.nodelist.append(road.rootnodes[0]);
                self.nodestack.append(road.rootnodes[1]);
                self.nodelist.append(road.nodes[0]);
                self.nodestack.append(road.nodes[1]);

                if road.rootnodes[0] > road.nodes[0]:
                    endscheck = [road.rootnodes[0],road.nodes[0]];
                elif road.rootnodes[0] < road.nodes[0]:
                    endscheck = [road.nodes[0],road.rootnodes[0]];
                for i in range(len(self.endslist)):
                    ends = self.endslist[i];
                    if (ends == endscheck)and(endscheck != road.nodes):
                        self.endslist.remove(ends);
                        del self.numlist[i];
                        del self.waitlist[i];
                        del self.firstdis[i];
                        self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                        break;

                if road.rootnodes[1] > road.nodes[1]:
                    endscheck = [road.rootnodes[1],road.nodes[1]];
                elif road.rootnodes[1] < road.nodes[1]:
                    endscheck = [road.nodes[1],road.rootnodes[1]];
                for i in range(len(self.endslist)):
                    ends = self.endslist[i];
                    if (ends == endscheck)and(endscheck != road.nodes):
                        self.endslist.remove(ends);
                        del self.numlist[i];
                        del self.waitlist[i];
                        del self.firstdis[i];
                        self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                        break;
                
                #print(road.rootnodes[1]);
                #print('thisround:'+str(self.nodelist));
                #print('thisround:'+str(self.nodestack));

                print('first nodes:'+str(road.nodes));

                self.restflt = self.restflt - road.firstdis;
                #print(self.restflt);
                #self.waitlist.remove(road);
                self.decendroads.remove(road);
                #self.firstdis.remove(road.firstdis);
                #del self.firstdis[road.RNO-1];
                thisserial = self.numlist.index(road.RNO);
                del self.firstdis[thisserial];
                del self.numlist[thisserial];
                print('delete:'+str(self.waitlist[thisserial].nodes));
                del self.waitlist[thisserial];
                del self.endslist[thisserial];
                print(len(self.waitlist));
                #self.iii = self.iii + 1;
                #print('num range:'+str(len(self.numlist)));

                secondbase1 = road.nodes[0];
                secondbase2 = road.nodes[1];

                diffdis = [self.firstdis[i] - road.minmindis[i] for i in range(len(self.firstdis))];

                #print('i range'+str(len(diffdis)));
                #print('list range'+str(len(self.waitlist)));
                for i in range(len(diffdis)):
                    self.waitlist[i].diffdis = diffdis[i];

                self.decenddiffroads = sorted(self.waitlist ,key = lambda road: road.diffdis, reverse = True);

                self.jjj = 0;
                while self.jjj < len(self.decenddiffroads):
                    roadnext = self.decenddiffroads[self.jjj];
                    roadnum = roadnext.RNO;
                    if roadnum < road.RNO:
                        serialnum = roadnum - 1;
                    else: 
                        serialnum = roadnum-2;
                    #print(serialnum);
                    
                    if road.minmindis[serialnum] <= self.restflt:
                        self.nodelist.append(road.nodeorder[serialnum,0]);
                        self.nodelist.append(roadnext.nodes[0]);
                        self.nodestack.append(road.nodeorder[serialnum,1]);
                        self.nodestack.append(roadnext.nodes[1]);

                        if road.nodeorder[serialnum,0] > roadnext.nodes[0]:
                            endscheck = [road.nodeorder[serialnum,0],roadnext.nodes[0]];
                        elif road.nodeorder[serialnum,0] < roadnext.nodes[0]:
                            endscheck = [roadnext.nodes[0],road.nodeorder[serialnum,0]];
                        for i in range(len(self.endslist)):
                            ends = self.endslist[i];
                            if (ends == endscheck)and(endscheck != roadnext.nodes):
                                self.endslist.remove(ends);
                                del self.numlist[i];
                                del self.waitlist[i];
                                del self.firstdis[i];
                                self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                                self.decenddiffroads = sorted(self.waitlist ,key = lambda road: road.diffdis, reverse = True);
                                break;

                        if road.nodeorder[serialnum,1] > roadnext.nodes[1]:
                            endscheck = [road.nodeorder[serialnum,1],roadnext.nodes[1]];
                        elif road.nodeorder[serialnum,1] < roadnext.nodes[1]:
                            endscheck = [roadnext.nodes[1],road.nodeorder[serialnum,1]];
                        for i in range(len(self.endslist)):
                            ends = self.endslist[i];
                            if (ends == endscheck)and(endscheck != roadnext.nodes):
                                self.endslist.remove(ends);
                                del self.numlist[i];
                                del self.waitlist[i];
                                del self.firstdis[i];
                                self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                                self.decenddiffroads = sorted(self.waitlist ,key = lambda road: road.diffdis, reverse = True);
                                break;

                        print('first:'+str(self.nodelist));
                        print('first:'+str(self.nodestack));

                        print('second nodes:'+str(roadnext.nodes));

                        self.restflt = self.restflt - road.minmindis[serialnum];

                        #print(len(self.waitlist));
                        #self.decendroads.remove(roadnext);
                        for i in range(len(self.decendroads)):
                            roadceck = self.decendroads[i];
                            if roadceck.RNO == roadnext.RNO:
                                self.decendroads.remove(roadceck);
                                break;

                        #self.waitlist.remove(roadnext);
                        #self.decenddiffroads.remove(roadnext);
                        thisserial = self.numlist.index(roadnext.RNO);
                        del self.firstdis[thisserial];
                        del self.numlist[thisserial];
                        print('delete:'+str(self.waitlist[thisserial].nodes));
                        del self.waitlist[thisserial];
                        del self.endslist[thisserial];
                        #self.firstdis.remove(roadnext.firstdis);
                        print(len(self.waitlist));
                        self.jjj= self.jjj + 1;

                        currentroad = roadnext;
                        self.highoptimize(road,currentroad,nodescoor);

                        #self.nodelist.append(self.nodestack.pop());
                        #self.nodelist.append(self.nodestack.pop());
                        while len(self.nodestack)-2 != 0:
                            self.nodelist.append(self.nodestack.pop());

                    else:
                        self.jjj = self.jjj + 1;
                        

            else:
                self.iii = self.iii + 1;
                #self.nodelist.append(self.nodestack.pop());
                #self.nodelist.append(self.nodestack.pop());
                if len(self.nodestack) != 0:
                    self.nodelist.append(self.nodestack.pop());
                    self.nodelist.append(self.nodestack.pop());

            if len(self.nodestack) != 0:
                self.nodelist.append(self.nodestack.pop());
                self.nodelist.append(self.nodestack.pop());


    def highoptimize(self,lastroad,currentroad,nodescoor):
        print('high!');
        lastwaitlen = self.waitlen;
        lastnode1 = lastroad.nodes[0];
        lastnode2 = lastroad.nodes[1];
        currentNO = currentroad.RNO;
        serialnum = 0;

        for road in self.waitlist:
            node1 = road.nodes[0];
            node2 = road.nodes[1];
            templen = road.length;
            
            disa1 = self.caldis(lastnode1,node1,nodescoor);
            disb1 = self.caldis(lastnode2,node1,nodescoor);
            disa2 = self.caldis(lastnode1,node2,nodescoor);
            disb2 = self.caldis(lastnode2,node2,nodescoor);

            min1 = min(disa1,disb1);
            min2 = min(disa2,disb2);

            if min1 == disa1:
                nodefi = lastnode1;
            elif min1 == disb1:
                nodefi = lastnode2;

            if min2 == disa2:
                nodesc = lastnode1;
            elif min2 == disb2:
                nodesc = lastnode2;

            road.lastdis = templen + min1 + min2;

            if road.RNO < currentNO:
                serialnum = road.RNO - 1;
            else:
                serialnum = road.RNO - 2;

            road.diffdis = road.lastdis - currentroad.minmindis[serialnum];

        self.decenddiffroads = sorted(self.waitlist, key = lambda road: road.diffdis, reverse = True);

        self.kkk = 0;
        while self.kkk < len(self.decenddiffroads):
            road = self.decenddiffroads[self.kkk];
            if currentroad.minmindis[serialnum] <= self.restflt:
                print(currentroad.minmindis[serialnum]);
                print(self.restflt);
                self.nodelist.append(currentroad.nodeorder[serialnum,0]);
                self.nodelist.append(road.nodes[0]);
                self.nodestack.append(currentroad.nodeorder[serialnum,1]);
                self.nodestack.append(road.nodes[1]);

                if currentroad.nodeorder[serialnum,0] > road.nodes[0]:
                    endscheck = [currentroad.nodeorder[serialnum,0],road.nodes[0]];
                elif currentroad.nodeorder[serialnum,0] < road.nodes[0]:
                    endscheck = [road.nodes[0],currentroad.nodeorder[serialnum,0]];
                for i in range(len(self.endslist)):
                    ends = self.endslist[i];
                    if (ends == endscheck)and(endscheck != road.nodes):
                        self.endslist.remove(ends);
                        del self.numlist[i];
                        del self.waitlist[i];
                        del self.firstdis[i];
                        self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                        self.decenddiffroads = sorted(self.waitlist ,key = lambda road: road.diffdis, reverse = True);
                        break;

                if currentroad.nodeorder[serialnum,1] > road.nodes[1]:
                    endscheck = [currentroad.nodeorder[serialnum,1],road.nodes[1]];
                elif currentroad.nodeorder[serialnum,1] < road.nodes[1]:
                    endscheck = [road.nodes[1],currentroad.nodeorder[serialnum,1]];
                for i in range(len(self.endslist)):
                    ends = self.endslist[i];
                    if (ends == endscheck)and(endscheck != road.nodes):
                        self.endslist.remove(ends);
                        del self.numlist[i];
                        del self.waitlist[i];
                        del self.firstdis[i];
                        self.decendroads = sorted(self.waitlist, key = lambda road: road.firstdis, reverse = True);
                        self.decenddiffroads = sorted(self.waitlist ,key = lambda road: road.diffdis, reverse = True);
                        break;

                print('high:'+str(self.nodelist));
                print('high:'+str(self.nodestack));

                print('high nodes:'+str(road.nodes));

                self.restflt = self.restflt - currentroad.minmindis[serialnum];

                #print(len(self.waitlist));
                #self.waitlist.remove(road);
                #self.decendroads.remove(road);
                for roadcheck in self.decendroads:
                    if roadcheck.RNO == road.RNO:
                        self.decendroads.remove(roadcheck);

                self.decenddiffroads.remove(road);
                thieserial = self.numlist.index(road.RNO);
                del self.firstdis[thieserial];
                del self.numlist[thieserial];
                print('delete:'+str(self.waitlist[thisserial].nodes));
                del self.waitlist[thieserial];
                del self.endslist[thieserial];
                #self.kkk = self.kkk + 1;
                print(len(self.waitlist));

                if lastwaitlen == len(self.waitlist):
                    self.failure = self.failure + 1;

                if self.failure < 5:
                    lastroadpie = currentroad;
                    currentroadpie = road;
                    self.highoptimize(lastroadpie,currentroadpie,nodescoor);
                else:
                    while len(self.nodestack)-2 != 0:
                        self.nodelist.append(self.nodestack.pop());

            else:
                self.kkk = self.kkk + 1;


    def calfirstdis(self,baseA,baseB,baseC,roads,nodescoor):
        self.firstdis = [];
        for road in roads:
            node1 = road.nodes[0];
            node2 = road.nodes[1];
            disa1 = road.caldis(node1,baseA,nodescoor);
            disa2 = road.caldis(node2,baseA,nodescoor);
            disb1 = road.caldis(node1,baseB,nodescoor);
            disb2 = road.caldis(node2,baseB,nodescoor);
            disc1 = road.caldis(node1,baseC,nodescoor);
            disc2 = road.caldis(node2,baseC,nodescoor);

            min1 = min(disa1,disb1,disc1);
            min2 = min(disa2,disb2,disc2);

            if min1 == disa1:
                road.rootnodes.append(baseA);
            elif min1 == disb1:
                road.rootnodes.append(baseB);
            elif min1 == disc1:
                road.rootnodes.append(baseC);

            if min2 == disa2:
                road.rootnodes.append(baseA);
            elif min2 == disb2:
                road.rootnodes.append(baseB);
            elif min2 == disc2:
                road.rootnodes.append(baseC);

            road.firstdis = road.length + min1 + min2;
            self.firstdis.append(road.firstdis);



    def caldis(self,node1,node2,nodescoor):
        nodes = nodescoor;
        r = node1-1;
        l = node2-1;

        xlat = nodes[r,0];
        xlon = nodes[r,1];
        xlat_rad = xlat*math.pi/180;
        xlon_rad = xlon*math.pi/180;

        ylat = nodes[l,0];
        ylon = nodes[l,1];
        ylat_rad = ylat*math.pi/180;
        ylon_rad = ylon*math.pi/180;

        dlat = abs(xlat-ylat);
        dlon = abs(xlon-ylon);

        dlat_rad = dlat*math.pi/180;
        dlon_rad = dlon*math.pi/180;

        s = 2*math.asin(math.sqrt(math.pow(math.sin(dlat_rad/2),2)+math.cos(xlat_rad)*math.cos(ylat_rad)*math.pow(math.sin(dlon_rad/2),2)));
        distance = s * 6378137/1000;

        return distance;


if __name__ == '__main__':
    distancesdata = [];
    with open('.\distances') as csvfile:
        csv_reader = csv.reader(csvfile);
        for row in csv_reader:
            distancesdata.append(row);

    distancesdata = [[float(x) for x in row] for row in distancesdata];
    distancesdata = np.array(distancesdata);

    #print(distancesdata.shape);

    nodescoor = [];
    current_path = os.path.dirname(__file__);
    with open(current_path+'/nodes') as csvfile:
        csv_reader = csv.reader(csvfile)
        for row in csv_reader:
            nodescoor.append(row);

    nodescoor = [[float(x) for x in row ] for row in nodescoor];
    nodescoor = np.array(nodescoor);

    #print(nodescoor.shape);

    roads = [];
    roadNO = 1;
    for i in range(0,48):
        for j in range(0,48):
            if ((j<i)and(distancesdata[i,j]!=0)):
                temproad = road(i+1,j+1,distancesdata[i,j],roadNO);
                roads.append(temproad);
                roadNO = roadNO + 1;

    #print(len(roads));

    for roadmin in roads:
        roadmin.findmindis(roads,nodescoor);

    findroute1 = Findroutes(10,14,47,roads,nodescoor);
    nodelist1 = findroute1.nodelist;
    print('result:'+str(nodelist1));
    print('total roads number:'+str(len(nodelist1)/2));

    print('final routes:');
    for route in findroute1.resultlist:
        print(route);

    #print('first distance:'+str(findroute1.firstdis));

    #print(str(distancesdata));
    #print(str(nodescoor));  
    exroad = road(1,2,distancesdata[0,1],1000);
    print(exroad.caldis(14,3,nodescoor)+exroad.caldis(3,14,nodescoor));
